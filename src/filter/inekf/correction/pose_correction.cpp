#include "drift/filter/inekf/correction/pose_correction.h"
#include <fstream>
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

  double std_pos = config_["noises"]["position_std"]
                       ? config_["noises"]["position_std"].as<double>()
                       : 0.1;
  double std_ori = config_["noises"]["orientation_std"]
                       ? config_["noises"]["orientation_std"].as<double>()
                       : 0.05;

  // Combined 6x6 covariance: [rotation (first 3), translation (last 3)]
  covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
  covariance_.topLeftCorner<3, 3>() = std_ori * std_ori * Eigen::Matrix3d::Identity();
  covariance_.bottomRightCorner<3, 3>() = std_pos * std_pos * Eigen::Matrix3d::Identity();

  t_diff_thres_ = config_["settings"]["correction_time_threshold"]
                      ? config_["settings"]["correction_time_threshold"].as<double>()
                      : 0.3;

  std::string est_pose_file = "/home/multy-surya/drift/log/full_pose_est_log.txt";
  est_pose_outfile_.open(est_pose_file);
  est_pose_outfile_.precision(dbl::max_digits10);
}

PoseCorrection::~PoseCorrection() {
  est_pose_outfile_.close();
}

const OdomQueuePtr PoseCorrection::get_sensor_data_buffer_ptr() const {
  return sensor_data_buffer_ptr_;
}

bool PoseCorrection::Correct(RobotState& state) {
  Eigen::VectorXd Z;
  Eigen::MatrixXd H, N;

  // Lock the sensor data buffer and check for new measurements
  sensor_data_buffer_mutex_ptr_->lock();
  if (sensor_data_buffer_ptr_->empty()) {
      sensor_data_buffer_mutex_ptr_->unlock();
      return false;
  }

  OdomMeasurementPtr measured_pose = sensor_data_buffer_ptr_->front();
  double t_diff = measured_pose->get_time() - state.get_propagate_time();

  // Skip measurements that are in the future
  if (t_diff >= 0) {
      sensor_data_buffer_mutex_ptr_->unlock();
      return false;
  }

  // Pop until we get to the most recent measurement within time threshold
  sensor_data_buffer_ptr_->pop();
  sensor_data_buffer_mutex_ptr_->unlock();

  if (t_diff < -t_diff_thres_) {
      while (t_diff < -t_diff_thres_) {
          sensor_data_buffer_mutex_ptr_->lock();
          if (sensor_data_buffer_ptr_->empty()) {
              sensor_data_buffer_mutex_ptr_->unlock();
              return false;
          }
          measured_pose = sensor_data_buffer_ptr_->front();
          sensor_data_buffer_ptr_->pop();
          sensor_data_buffer_mutex_ptr_->unlock();

          t_diff = measured_pose->get_time() - state.get_propagate_time();
      }
  }

  // Update state time to match the measurement
  state.set_time(measured_pose->get_time());

  int dimP = state.dimP();
  const Eigen::Matrix3d& R = state.get_rotation();
  const Eigen::Vector3d& p = state.get_position();
  const Eigen::Matrix4d& T_meas = measured_pose->get_transformation();

  const Eigen::Vector3d p_meas = T_meas.block<3,1>(0,3);
  const Eigen::Matrix3d R_meas = T_meas.block<3,3>(0,0);

  // Orientation frame vectors from measured rotation
  Eigen::Vector3d y1 = R_meas.col(0);
  Eigen::Vector3d y2 = R_meas.col(1);
  Eigen::Vector3d y3 = R_meas.col(2);

  // Reference body frame vectors (assume canonical basis)
  Eigen::Vector3d b1 = Eigen::Vector3d::UnitX();
  Eigen::Vector3d b2 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d b3 = Eigen::Vector3d::UnitZ();

  // Build residual vector Z (size 12)
  Z.resize(12);
  Z.segment<3>(0) = R.transpose() * (p_meas - p);
  Z.segment<3>(3) = R.transpose() * y1 - b1;
  Z.segment<3>(6) = R.transpose() * y2 - b2;
  Z.segment<3>(9) = R.transpose() * y3 - b3;

  // Build measurement Jacobian H (12 x dimP)
  H = Eigen::MatrixXd::Zero(12, dimP);
  H.block<3,3>(0,6) = Eigen::Matrix3d::Identity(); // d(position residual)/d(p)

  H.block<3,3>(3,0) = -math::lie_group::skew(b1); // d(y1 residual)/d(θ)
  H.block<3,3>(6,0) = -math::lie_group::skew(b2);
  H.block<3,3>(9,0) = -math::lie_group::skew(b3);

  // Noise covariance matrix N (12 x 12)
  N = Eigen::MatrixXd::Zero(12, 12);
  Eigen::Matrix3d pos_cov_ = R.transpose() * covariance_.bottomRightCorner<3, 3>() * R;
  Eigen::Matrix3d ori_cov_ = R.transpose() * covariance_.topLeftCorner<3, 3>() * R;
  N.block<3,3>(0,0) = pos_cov_;                // Position
  N.block<3,3>(3,3) = ori_cov_;                // Orientation vector 1
  N.block<3,3>(6,6) = ori_cov_;                // Orientation vector 2
  N.block<3,3>(9,9) = ori_cov_;                // Orientation vector 3

  // Perform correction using Left-Invariant EKF
  if (Z.rows() > 0) {
    CorrectLeftInvariant(Z, H, N, state, error_type_);
  }

  // Logging
  Eigen::Vector3d propagate_position = p;
  Eigen::Vector3d est_position = state.get_position();

  est_pose_outfile_ << "Propagated: " << propagate_position.transpose() << " "
                    << "Measured: " << p_meas.transpose() << " "
                    << "Estimated: " << est_position.transpose() << std::endl << std::flush;

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

  state.set_position(measured_pose->get_transformation().block<3, 1>(0, 3));
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
