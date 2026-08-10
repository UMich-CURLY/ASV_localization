#include "drift/filter/inekf/correction/pose_correction.h"
#include <stdexcept>
#include <vector>
using namespace std;
using namespace math::lie_group;

namespace filter::inekf {

PoseCorrection::PoseCorrection(
    OdomQueuePtr sensor_data_buffer_ptr,
    std::shared_ptr<std::mutex> sensor_data_buffer_mutex_ptr,
    const ErrorType& error_type,
    const std::string& yaml_filepath)
    : Correction::Correction(sensor_data_buffer_mutex_ptr),
      sensor_data_buffer_ptr_(sensor_data_buffer_ptr),
      error_type_(error_type) {
  correction_type_ = CorrectionType::POSE;

  // Load configuration settings from YAML file
  cout << "Loading pose correction config from " << yaml_filepath << endl;
  YAML::Node config_ = YAML::LoadFile(yaml_filepath);

  Eigen::Vector3d std_pos =
      Eigen::Vector3d::Constant(config_["noises"]["position_std"]
                                    ? config_["noises"]["position_std"].as<double>()
                                    : 0.1);
  if (config_["noises"]["position_std_xyz"]) {
    const auto std_pos_xyz =
        config_["noises"]["position_std_xyz"].as<std::vector<double>>();
    if (std_pos_xyz.size() != 3) {
      throw std::runtime_error(
          "pose_correction position_std_xyz must have exactly 3 values");
    }
    std_pos = Eigen::Vector3d(std_pos_xyz[0], std_pos_xyz[1], std_pos_xyz[2]);
  }
  if ((std_pos.array() <= 0.0).any()) {
    throw std::runtime_error("pose_correction position std values must be positive");
  }
  double std_ori = config_["noises"]["orientation_std"]
                       ? config_["noises"]["orientation_std"].as<double>()
                       : 0.05;

  // Combined 6x6 covariance: [rotation (first 3), translation (last 3)]
  covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
  covariance_.topLeftCorner<3, 3>() = std_ori * std_ori * Eigen::Matrix3d::Identity();
  covariance_.bottomRightCorner<3, 3>() =
      std_pos.array().square().matrix().asDiagonal();

  t_diff_thres_ = config_["settings"]["correction_time_threshold"]
                      ? config_["settings"]["correction_time_threshold"].as<double>()
                      : 0.3;
  fix_z_ = config_["settings"]["fix_z"]
               ? config_["settings"]["fix_z"].as<bool>()
               : false;
  fixed_z_value_ = config_["settings"]["fixed_z_value"]
                       ? config_["settings"]["fixed_z_value"].as<double>()
                       : 0.0;
  decouple_prior_covariance_ = config_["settings"]["decouple_prior_covariance"]
                                    ? config_["settings"]["decouple_prior_covariance"].as<bool>()
                                    : true;
}

PoseCorrection::~PoseCorrection() {
}

const OdomQueuePtr PoseCorrection::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

bool PoseCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z;
  Eigen::MatrixXd H, N;

  OdomMeasurementPtr measured_pose;
  {
    std::lock_guard<std::mutex> lock(*sensor_data_buffer_mutex_ptr_);

    while (!sensor_data_buffer_ptr_->empty()) {
      const auto candidate = sensor_data_buffer_ptr_->front();
      const double t_diff =
          candidate->get_time() - state.get_propagate_time();

      // Keep future data queued until propagation reaches its timestamp.
      if (t_diff > 0.0) {
        return false;
      }

      sensor_data_buffer_ptr_->pop();

      // Discard measurements that are too old to correct the current state.
      if (t_diff < -t_diff_thres_) {
        continue;
      }

      measured_pose = candidate;
      break;
    }
  }

  if (!measured_pose) {
    return false;
  }

  // Update state time to match the measurement
  state.set_time(measured_pose->get_time());

  int dimP = state.dimP();
  const Eigen::Matrix4d& T_meas = measured_pose->get_transformation();

  Eigen::Vector3d p_meas = T_meas.block<3,1>(0,3);
  if (fix_z_) {
    p_meas.z() = fixed_z_value_;
  }
  const Eigen::Matrix3d R_meas = T_meas.block<3,3>(0,0);

  // Orientation frame vectors from measured rotation
  Eigen::Vector3d y1 = R_meas.col(0);
  Eigen::Vector3d y2 = R_meas.col(1);
  Eigen::Vector3d y3 = R_meas.col(2);

  // Reference body frame vectors (assume canonical basis)
  Eigen::Vector3d b1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d b2 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d b3 = Eigen::Vector3d::UnitZ();

  const Eigen::Matrix3d R = state.get_rotation();
  const Eigen::Vector3d p = state.get_position();

  const int total_rows = 3 + 9;

  Eigen::Vector3d Z_pos = R.transpose() * (p_meas - p);
  Eigen::MatrixXd H_pos = Eigen::MatrixXd::Zero(3, dimP);
  H_pos.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d N_pos = R.transpose() * covariance_.bottomRightCorner<3, 3>() * R;

  Z.resize(total_rows);
  H = Eigen::MatrixXd::Zero(total_rows, dimP);
  N = Eigen::MatrixXd::Zero(total_rows, total_rows);
  Z.segment<3>(0) = Z_pos;
  H.block(0, 0, 3, dimP) = H_pos;
  N.block<3,3>(0,0) = N_pos;

  Eigen::Matrix3d ori_cov_ = R.transpose() * covariance_.topLeftCorner<3, 3>() * R;
  Z.segment<3>(3) = R.transpose() * y1 - b1;
  Z.segment<3>(6) = R.transpose() * y2 - b2;
  Z.segment<3>(9) = R.transpose() * y3 - b3;
  H.block<3,3>(3,0) = -math::lie_group::skew(b1);
  H.block<3,3>(6,0) = -math::lie_group::skew(b2);
  H.block<3,3>(9,0) = -math::lie_group::skew(b3);
  N.block<3,3>(3,3) = ori_cov_;
  N.block<3,3>(6,6) = ori_cov_;
  N.block<3,3>(9,9) = ori_cov_;

  // Perform the joint 12-row correction using the Left-Invariant EKF. See
  // decouple_prior_covariance_'s doc comment in the header: when true (the
  // default), the position-rotation/position-bias prior cross-covariance
  // is excluded from this one gain computation, so GPS position cannot
  // perturb attitude/bias through that correlation.
  if (decouple_prior_covariance_) {
    int dimTheta = state.dimTheta();
    Eigen::VectorXd group_position = Eigen::VectorXd::Zero(dimP);
    group_position.segment(6, 3).setOnes();
    Eigen::VectorXd group_rotation_bias = Eigen::VectorXd::Zero(dimP);
    group_rotation_bias.segment(0, 3).setOnes();
    group_rotation_bias.segment(dimP - dimTheta, dimTheta).setOnes();

    CorrectLeftInvariant(Z, H, N, state, error_type_,
                          group_position, group_rotation_bias);
  } else {
    CorrectLeftInvariant(Z, H, N, state, error_type_);
  }

  return true;
}

bool PoseCorrection::initialize(RobotState& state) {
  if (sensor_data_buffer_ptr_->empty()) {
    return false;
  }

  sensor_data_buffer_mutex_ptr_->lock();
  while (sensor_data_buffer_ptr_->size() > 1) {
    sensor_data_buffer_ptr_->pop();
  }
  OdomMeasurementPtr measured_pose = sensor_data_buffer_ptr_->front();
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_->unlock();

  Eigen::Vector3d initial_position =
      measured_pose->get_transformation().block<3, 1>(0, 3);
  if (fix_z_) {
    initial_position.z() = fixed_z_value_;
  }
  state.set_position(initial_position);
  state.set_rotation(measured_pose->get_transformation().block<3, 3>(0, 0));
  state.set_time(measured_pose->get_time());

  return true;
}

void PoseCorrection::clear() {
  sensor_data_buffer_mutex_ptr_->lock();
  while (!sensor_data_buffer_ptr_->empty()) {
    sensor_data_buffer_ptr_->pop();
  }
  sensor_data_buffer_mutex_ptr_->unlock();
}

}  // namespace filter::inekf
