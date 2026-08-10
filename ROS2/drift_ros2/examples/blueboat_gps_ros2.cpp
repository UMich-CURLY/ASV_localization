/* ----------------------------------------------------------------------------
 * Copyright 2026, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   blueboat_gps_ros2.cpp
 *  @author Tim
 *  @brief  Real BlueBoat setup for GPS + IMU localization
 *          using DRIFT InEKF propagation and GPS position correction.
 *  @date   July 6, 2026
 **/

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>

#include "communication/ros2_publisher.h"
#include "communication/ros2_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace estimator;
using namespace state;
using namespace std;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

double NormalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double QuaternionYawFromComponents(
    const double w,
    const double x,
    const double y,
    const double z) {
  const double norm_sq = w * w + x * x + y * y + z * z;
  if (norm_sq <= 1e-12) {
    return 0.0;
  }

  const double inv_norm = 1.0 / std::sqrt(norm_sq);
  const double wn = w * inv_norm;
  const double xn = x * inv_norm;
  const double yn = y * inv_norm;
  const double zn = z * inv_norm;

  return std::atan2(2.0 * (wn * zn + xn * yn),
                    1.0 - 2.0 * (yn * yn + zn * zn));
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion& q) {
  return QuaternionYawFromComponents(q.w, q.x, q.y, q.z);
}

double QuaternionYaw(const Eigen::Quaterniond& q) {
  return QuaternionYawFromComponents(q.w(), q.x(), q.y(), q.z());
}

double StampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

struct InitialAlignmentSamples {
  std::mutex mutex;
  bool have_pose = false;
  bool have_imu = false;
  double pose_stamp = 0.0;
  double imu_stamp = 0.0;
  double pose_yaw = 0.0;
  double imu_body_yaw = 0.0;
  double last_used_pose_stamp = -1.0;
  std::vector<double> yaw_differences;
};

bool WaitForInitialWorldAlignment(
    const rclcpp::Node::SharedPtr& node,
    rclcpp::executors::MultiThreadedExecutor& executor,
    const std::string& pose_topic,
    const std::string& imu_topic,
    const Eigen::Quaterniond& q_imu_from_body,
    const double timeout_sec,
    const double max_time_difference_sec,
    const std::size_t required_sample_count,
    double& world_alignment_yaw_rad) {
  auto samples = std::make_shared<InitialAlignmentSamples>();

  const auto try_add_sample =
      [samples, max_time_difference_sec]() {
        if (!samples->have_pose || !samples->have_imu ||
            samples->pose_stamp == samples->last_used_pose_stamp ||
            std::abs(samples->pose_stamp - samples->imu_stamp) >
                max_time_difference_sec) {
          return;
        }

        samples->yaw_differences.push_back(NormalizeAngle(
            samples->pose_yaw - samples->imu_body_yaw));
        samples->last_used_pose_stamp = samples->pose_stamp;
      };

  auto pose_sub =
      node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          pose_topic, rclcpp::SensorDataQoS(),
          [samples, try_add_sample](
              const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            const auto& q = msg->pose.pose.orientation;
            const double norm_sq =
                q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
            if (norm_sq <= 1e-12) {
              return;
            }

            std::lock_guard<std::mutex> lock(samples->mutex);
            samples->pose_stamp = StampToSeconds(msg->header.stamp);
            samples->pose_yaw = QuaternionYaw(q);
            samples->have_pose = true;
            try_add_sample();
          });

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [samples, try_add_sample, q_imu_from_body](
          const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (msg->orientation_covariance[0] < 0.0) {
          return;
        }

        Eigen::Quaterniond q_ahrs_from_imu(
            msg->orientation.w, msg->orientation.x,
            msg->orientation.y, msg->orientation.z);
        if (q_ahrs_from_imu.squaredNorm() <= 1e-12) {
          return;
        }
        q_ahrs_from_imu.normalize();

        const Eigen::Quaterniond q_ahrs_from_body =
            q_ahrs_from_imu * q_imu_from_body;

        std::lock_guard<std::mutex> lock(samples->mutex);
        samples->imu_stamp = StampToSeconds(msg->header.stamp);
        samples->imu_body_yaw = QuaternionYaw(q_ahrs_from_body);
        samples->have_imu = true;
        try_add_sample();
      });

  RCLCPP_INFO(
      node->get_logger(),
      "Waiting for %zu synchronized samples from %s and %s.",
      required_sample_count, pose_topic.c_str(), imu_topic.c_str());

  rclcpp::Rate rate(50);
  const auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    executor.spin_some();

    {
      std::lock_guard<std::mutex> lock(samples->mutex);
      if (samples->yaw_differences.size() >= required_sample_count) {
        break;
      }
    }

    if (timeout_sec > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      const std::chrono::duration<double> elapsed = now - start_time;
      if (elapsed.count() >= timeout_sec) {
        break;
      }
    }

    rate.sleep();
  }

  std::lock_guard<std::mutex> lock(samples->mutex);
  if (samples->yaw_differences.size() < required_sample_count) {
    return false;
  }

  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const double yaw_difference : samples->yaw_differences) {
    sin_sum += std::sin(yaw_difference);
    cos_sum += std::cos(yaw_difference);
  }
  world_alignment_yaw_rad = std::atan2(sin_sum, cos_sum);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  /// Initialize ROS 2 node.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("asv_gps_ros2");

  /// ROS 2 callbacks are handled by the executor while the estimator runs in
  /// its own loop below.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  /// Load ROS communication and estimator configuration.
  std::string file{__FILE__};
  std::string project_dir{file.substr(0, file.rfind("ROS2/drift_ros2/examples/"))};
  std::string ros_config_file =
      project_dir + "/ROS2/drift_ros2/config/blueboat_real/ros_comm.yaml";
  std::string blueboat_config_dir = project_dir + "/config/blueboat_real";
  ///error message if the specified config doesn't exist
  if (!std::filesystem::exists(ros_config_file)) {
    RCLCPP_FATAL(node->get_logger(),
                 "ROS communication config does not exist: %s",
                 ros_config_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  const YAML::Node config = YAML::LoadFile(ros_config_file);        //we loaded ros_comm.yaml file
  const YAML::Node subscribers_config = config["subscribers"];      //we loaded the imu & gps topics from ros_comm.yaml

  /// blueboat estimator config files
  const std::string inekf_estimator_config =
      blueboat_config_dir + "/inekf_estimator.yaml";
  const std::string imu_propagation_config =
      blueboat_config_dir + "/imu_propagation.yaml";
  const std::string pose_correction_config =
      blueboat_config_dir + "/pose_correction.yaml";

  /// Read sensor topics and body-frame GPS antenna transform from YAML.
  const std::string imu_propagation_topic = subscribers_config["imu_propagation_topic"].as<std::string>();
  const std::string imu_orientation_topic = subscribers_config["imu_orientation_topic"].as<std::string>();
  const std::string gps_topic = subscribers_config["gps_topic"].as<std::string>();
  const auto translation_gpssrc2body =
      subscribers_config["translation_gps_source_to_body"].as<std::vector<double>>();   //this is the translation from gps main antenna to the body frame

  std::vector<double> rotation_imu2body{1.0, 0.0, 0.0, 0.0};    //the imu default rotation is aligned with base_link unless otehrwise specified in imu_propagation.yaml
  std::vector<double> rotation_gpssrc2body{1.0, 0.0, 0.0, 0.0};  //the gps default rotation is aligned with base_link unless otherwise specified in ros_comm.yaml

  // here we can get a user-defined rotation wrt body from the imu_propagation.yaml file, if it exist
  const YAML::Node imu_config = YAML::LoadFile(imu_propagation_config);
  if (imu_config["settings"] && imu_config["settings"]["rotation_imu2body"]) {
    rotation_imu2body =
        imu_config["settings"]["rotation_imu2body"].as<std::vector<double>>();
  }

  // here we can get a user-defined rotation wrt body from the ros_comm.yaml file, if it exist
  if (subscribers_config["rotation_gps_source_to_body"]) {
    rotation_gpssrc2body =
        subscribers_config["rotation_gps_source_to_body"].as<std::vector<double>>();
  }

  /// Create ROS subscribers.
  auto ros_sub = std::make_shared<ros_wrapper::ROSSubscriber>(node);

  const YAML::Node heading_config = config["heading_correction"];
  if (heading_config) {
    const bool use_manual_world_alignment =
        heading_config["use_manual_world_alignment"]
            ? heading_config["use_manual_world_alignment"].as<bool>()
            : true;

    if (use_manual_world_alignment) {
      const double manual_yaw_deg =
          heading_config["manual_world_alignment_yaw_deg"]
              ? heading_config["manual_world_alignment_yaw_deg"].as<double>()
              : 0.0;
      RCLCPP_INFO(node->get_logger(), "Using manual world alignment.");
      ros_sub->SetWorldAlignmentYaw(manual_yaw_deg * kDegToRad);
    } else {
      const std::string gps_pose_topic =
          heading_config["gps_pose_topic"]
              ? heading_config["gps_pose_topic"].as<std::string>()
              : "/pose";
      const double timeout_sec =
          heading_config["initial_alignment_timeout_sec"]
              ? heading_config["initial_alignment_timeout_sec"].as<double>()
              : 5.0;
      const double max_time_difference_sec =
          heading_config["initial_alignment_max_time_difference_sec"]
              ? heading_config["initial_alignment_max_time_difference_sec"]
                    .as<double>()
              : 0.05;
      const int configured_sample_count =
          heading_config["initial_alignment_sample_count"]
              ? heading_config["initial_alignment_sample_count"].as<int>()
              : 5;
      const std::size_t sample_count =
          configured_sample_count > 0
              ? static_cast<std::size_t>(configured_sample_count)
              : 1;

      if (rotation_imu2body.size() != 4) {
        RCLCPP_FATAL(node->get_logger(),
                     "rotation_imu2body must contain [w, x, y, z].");
        rclcpp::shutdown();
        return 1;
      }
      Eigen::Quaterniond q_body_from_imu(
          rotation_imu2body[0], rotation_imu2body[1],
          rotation_imu2body[2], rotation_imu2body[3]);
      if (q_body_from_imu.squaredNorm() <= 1e-12) {
        RCLCPP_FATAL(node->get_logger(),
                     "rotation_imu2body must be a nonzero quaternion.");
        rclcpp::shutdown();
        return 1;
      }
      q_body_from_imu.normalize();

      double world_alignment_yaw_rad = 0.0;
      if (!WaitForInitialWorldAlignment(
              node, executor, gps_pose_topic, imu_orientation_topic,
              q_body_from_imu.inverse(), timeout_sec,
              max_time_difference_sec, sample_count,
              world_alignment_yaw_rad)) {
        RCLCPP_FATAL(
            node->get_logger(),
            "Timed out waiting for synchronized world-alignment samples. "
            "Set use_manual_world_alignment to true to use the configured yaw.");
        rclcpp::shutdown();
        return 1;
      }
      RCLCPP_INFO(node->get_logger(),
                  "Using automatically initialized world alignment.");
      ros_sub->SetWorldAlignmentYaw(world_alignment_yaw_rad);
    }
  }

  /// Add a subscriber for IMU data and get its queue and mutex.
  auto qimu_and_mutex = ros_sub->AddIMUSubscriber(imu_propagation_topic);
  auto qimu = qimu_and_mutex.first;
  auto qimu_mutex = qimu_and_mutex.second;


  /// Add a combined GPS+IMU pose subscriber and get its pose queue and mutex.
  /// The callback converts NavSatFix to local ENU, applies the GPS antenna
  /// offset to recover the body pose, and uses IMU orientation for heading.
  auto qpose_and_mutex = ros_sub->AddGPSIMU2PoseSubscriber(
      gps_topic, imu_orientation_topic, translation_gpssrc2body, rotation_gpssrc2body,
      rotation_imu2body);
  auto qpose = qpose_and_mutex.first;
  auto qpose_mutex = qpose_and_mutex.second;

  /// Define the InEKF error type and create the state estimator.
  const filter::inekf::ErrorType error_type = filter::inekf::RightInvariant;
  InekfEstimator inekf_estimator(error_type, inekf_estimator_config);

  /// Add propagation and correction methods to the estimator.
  /// For the real blueboat, this is IMU propagation plus IMU+GPS combine pose
  /// correction with a known off-center antenna lever arm.
  inekf_estimator.add_imu_propagation(qimu, qimu_mutex, imu_propagation_config);
  inekf_estimator.add_pose_correction(qpose, qpose_mutex, pose_correction_config);

  // inekf_estimator.add_position_correction(qpos, qpos_mutex, position_correction_config); ///need to modify this for true 12x1 pose correction from IMU+GPS

  /// Get the robot state queue and mutex from the state estimator.
  RobotStateQueuePtr robot_state_queue_ptr =
      inekf_estimator.get_robot_state_queue_ptr();
  std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr =
      inekf_estimator.get_robot_state_queue_mutex_ptr();

  /// Create a ROS publisher and start the publishing thread.
  auto ros_pub = std::make_shared<ros_wrapper::ROSPublisher>(
      node, robot_state_queue_ptr, robot_state_queue_mutex_ptr, ros_config_file);
  ros_pub->StartPublishingThread();

  /// Run the state estimator. Startup is staged:
  ///   1. InitBias consumes the configured number of static IMU samples.
  ///   2. InitState waits for the first GPS correction source and initializes X.
  ///   3. RunOnce propagates on IMU and applies any available GPS corrections.
  std::thread estimator_thread([&]() {
    rclcpp::Rate rate(5000);
    while (rclcpp::ok()) {
      if (inekf_estimator.is_enabled()) {
        inekf_estimator.RunOnce();
      } else {
        if (inekf_estimator.BiasInitialized()) {
          inekf_estimator.InitState();
        } else {
          inekf_estimator.InitBias();
        }
      }
      rate.sleep();
    }
  });

  executor.spin();

  if (estimator_thread.joinable()) {
    estimator_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
