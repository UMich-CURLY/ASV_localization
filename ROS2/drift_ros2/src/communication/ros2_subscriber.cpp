#include "communication/ros2_subscriber.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <novatel_oem7_msgs/msg/bestgnsspos.hpp>

namespace ros_wrapper {

ROSSubscriber::ROSSubscriber(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {}

ROSSubscriber::~ROSSubscriber() {
  if (thread_started_ && subscribing_thread_.joinable()) {
    subscribing_thread_.join();
  }
  subscriber_list_.clear();
  imu_queue_list_.clear();
  position_queue_list_.clear();
}

IMUQueuePair ROSSubscriber::AddIMUSubscriber(const std::string& topic_name) {
  RCLCPP_INFO(node_->get_logger(), "Subscribing to IMU topic: %s", topic_name.c_str());

  IMUQueuePtr imu_queue_ptr = std::make_shared<std::queue<std::shared_ptr<ImuMeasurement<double>>>>();
  auto mutex = std::make_shared<std::mutex>();

  auto callback = [this, mutex, imu_queue_ptr](const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    this->IMUCallback(imu_msg, mutex, imu_queue_ptr);
  };

  subscriber_list_.push_back(
      node_->create_subscription<sensor_msgs::msg::Imu>(topic_name, 1000, callback));
  imu_queue_list_.push_back(imu_queue_ptr);

  return {imu_queue_ptr, mutex};
}

PositionQueuePair ROSSubscriber::AddOdom2PositionSubscriber(
    const std::string& topic_name,
    const std::vector<double>& translation_odomsrc2body,
    const std::vector<double>& rotation_odomsrc2body) {
  RCLCPP_INFO(node_->get_logger(), "Subscribing to odometry topic: %s", topic_name.c_str());

  auto position_queue_ptr = std::make_shared<OdomQueue>();
  auto mutex = std::make_shared<std::mutex>();

  Eigen::Quaterniond orientation_quat(rotation_odomsrc2body[0], rotation_odomsrc2body[1],
                                      rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
  odom_src_to_body_ = Eigen::Matrix4d::Identity();
  odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_odomsrc2body.data());

  auto callback = [this, mutex, position_queue_ptr](const nav_msgs::msg::Odometry::SharedPtr msg) {
    Odom2PositionCallback(msg, mutex, position_queue_ptr);
  };

  subscriber_list_.push_back(
      node_->create_subscription<nav_msgs::msg::Odometry>(topic_name, 1000, callback));

  position_queue_list_.push_back(position_queue_ptr);
  return {position_queue_ptr, mutex};
}

PositionQueuePair ROSSubscriber::AddGPS2PositionSubscriber(
    const std::string& topic_name,
    const std::vector<double>& translation_gpssrc2body,
    const std::vector<double>& rotation_gpssrc2body) {
  RCLCPP_INFO(node_->get_logger(), "Subscribing to GPS topic: %s", topic_name.c_str());

  auto position_queue_ptr = std::make_shared<OdomQueue>();
  auto mutex = std::make_shared<std::mutex>();

  Eigen::Quaterniond orientation_quat(rotation_gpssrc2body[0], rotation_gpssrc2body[1],
                                      rotation_gpssrc2body[2], rotation_gpssrc2body[3]);

  gps_src_to_body_ = Eigen::Matrix4d::Identity();
  gps_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  gps_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_gpssrc2body.data());

  auto callback = [this, mutex, position_queue_ptr](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!reference_initialized_) {
      reference_position_ << msg->latitude, msg->longitude, msg->altitude;
      reference_initialized_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "GPS reference position initialized: [%.8f, %.8f, %.3f]",
                  reference_position_.x(), reference_position_.y(), reference_position_.z());
    }

    if (reference_initialized_) {
      GPS2PositionCallback(msg, mutex, position_queue_ptr);
    }
  };

  subscriber_list_.push_back(
      node_->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name, 1000, callback));

  position_queue_list_.push_back(position_queue_ptr);

  return {position_queue_ptr, mutex};
}

PoseQueuePair ROSSubscriber::AddGPSIMU2PoseSubscriber(
    const std::string& gps_topic_name,
    const std::string& imu_topic_name,
    const std::vector<double>& translation_gpssrc2body,
    const std::vector<double>& rotation_gpssrc2body,
    double heading_offset_rad) {
  RCLCPP_INFO(node_->get_logger(), "Subscribing to GPS: %s and IMU: %s",
              gps_topic_name.c_str(), imu_topic_name.c_str());

  auto pose_queue_ptr = std::make_shared<OdomQueue>();
  auto mutex = std::make_shared<std::mutex>();

  Eigen::Quaterniond orientation_quat(rotation_gpssrc2body[0], rotation_gpssrc2body[1],
                                      rotation_gpssrc2body[2], rotation_gpssrc2body[3]);

  gps_src_to_body_ = Eigen::Matrix4d::Identity();
  gps_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  gps_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_gpssrc2body.data());
  heading_offset_quat_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(heading_offset_rad, Eigen::Vector3d::UnitZ()));

  auto latest_orientation = std::make_shared<geometry_msgs::msg::Quaternion>();

  auto imu_callback = [this, latest_orientation](const sensor_msgs::msg::Imu::SharedPtr msg) {
    *latest_orientation = msg->orientation;
    imu_orientation_ready_ = true;
  };

  subscriber_list_.push_back(
      node_->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 1000, imu_callback));

  auto gps_callback = [this, mutex, pose_queue_ptr, latest_orientation](
                          const novatel_oem7_msgs::msg::BESTGNSSPOS::SharedPtr n) {
    auto fix = std::make_shared<sensor_msgs::msg::NavSatFix>();
    fix->header = n->header;
    fix->latitude = n->lat;
    fix->longitude = n->lon;
    fix->altitude = n->hgt + n->undulation;

    if (!reference_initialized_) {
      reference_position_ << fix->latitude, fix->longitude, fix->altitude;
      reference_initialized_ = true;
      RCLCPP_INFO(node_->get_logger(),
                  "GPS reference position initialized: [%.8f, %.8f, %.3f]",
                  reference_position_.x(), reference_position_.y(), reference_position_.z());
    }

    if (!reference_initialized_ || !imu_orientation_ready_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Reference or IMU orientation not initialized, skipping GPS/IMU fusion");
      return;
    }

    const geometry_msgs::msg::Quaternion& ros_q = *latest_orientation;
    const Eigen::Quaterniond current_q(ros_q.w, ros_q.x, ros_q.y, ros_q.z);

    GPSIMU2PoseCallback(fix, mutex, current_q, pose_queue_ptr);
  };

  subscriber_list_.push_back(node_->create_subscription<novatel_oem7_msgs::msg::BESTGNSSPOS>(
      gps_topic_name, 1000, gps_callback));

  return {pose_queue_ptr, mutex};
}

void ROSSubscriber::StartSubscribingThread() {
  if (thread_started_) {
    RCLCPP_WARN(node_->get_logger(), "Subscribing thread already running");
    return;
  }

  subscribing_thread_ = std::thread([this]() { this->RosSpin(); });
  thread_started_ = true;
}

void ROSSubscriber::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                std::shared_ptr<std::mutex> mutex,
                                IMUQueuePtr imu_queue) {
  auto imu_measurement = std::make_shared<ImuMeasurement<double>>();

  imu_measurement->set_header(
      imu_msg->header.stamp.sec,
      imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9,
      imu_msg->header.frame_id);

  imu_measurement->set_angular_velocity(
      imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

  imu_measurement->set_lin_acc(
      imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);

  const Eigen::Vector4d quat(imu_msg->orientation.w,
                             imu_msg->orientation.x,
                             imu_msg->orientation.y,
                             imu_msg->orientation.z);

  if (quat.norm() != 0.0) {
    imu_measurement->set_quaternion(
        imu_msg->orientation.w, imu_msg->orientation.x,
        imu_msg->orientation.y, imu_msg->orientation.z);
  }

  std::lock_guard<std::mutex> lock(*mutex);
  imu_queue->push(imu_measurement);
}

void ROSSubscriber::Odom2PositionCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    std::shared_ptr<std::mutex> position_mutex,
    OdomQueuePtr position_queue) {
  auto position_measurement = std::make_shared<OdomMeasurement>();
  const Eigen::Vector3d translation(
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);

  Eigen::Matrix4d curr_transformation = Eigen::Matrix4d::Identity();
  curr_transformation.block<3, 1>(0, 3) = translation;

  Eigen::Matrix4d transformed_pose = odom_src_to_body_.inverse() * curr_transformation;
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);
  if (!transformed_translation.allFinite()) {
    return;
  }

  position_measurement->set_header(
      odom_msg->header.stamp.sec,
      odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1e9,
      odom_msg->header.frame_id);
  position_measurement->set_translation(transformed_translation);
  position_measurement->set_transformation();

  std::lock_guard<std::mutex> lock(*position_mutex);
  position_queue->push(position_measurement);
}

void ROSSubscriber::GPS2PositionCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    std::shared_ptr<std::mutex> position_mutex,
    OdomQueuePtr position_queue) {
  if (!reference_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Reference position not initialized yet, skipping GPS position");
    return;
  }

  auto position_measurement = std::make_shared<OdomMeasurement>();

  measurement::NavSatMeasurement<double> navsat_measurement;
  navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

  Eigen::Matrix<double, 3, 1> enu_translation =
      navsat_measurement.get_enu(reference_position_.x(),
                                 reference_position_.y(),
                                 reference_position_.z());

  Eigen::Matrix4d enu_transformation = Eigen::Matrix4d::Identity();
  enu_transformation.block<3, 1>(0, 3) = enu_translation;

  Eigen::Matrix4d transformed_pose = gps_src_to_body_.inverse() * enu_transformation;
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  position_measurement->set_header(
      gps_msg->header.stamp.sec,
      gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec / 1e9,
      gps_msg->header.frame_id);

  position_measurement->set_translation(transformed_translation);
  position_measurement->set_transformation();

  std::lock_guard<std::mutex> lock(*position_mutex);
  position_queue->push(position_measurement);
}

void ROSSubscriber::GPSIMU2PoseCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    std::shared_ptr<std::mutex> pose_mutex,
    const Eigen::Quaterniond& imu_orientation,
    OdomQueuePtr pose_queue) {
  if (!reference_initialized_) {
    RCLCPP_WARN(node_->get_logger(), "Reference position not initialized yet, skipping fused pose");
    return;
  }

  auto pose_measurement = std::make_shared<OdomMeasurement>();

  measurement::NavSatMeasurement<double> navsat_measurement;
  navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
  Eigen::Vector3d enu_translation = navsat_measurement.get_enu(
      reference_position_.x(), reference_position_.y(), reference_position_.z());

  Eigen::Matrix4d enu_transformation = Eigen::Matrix4d::Identity();
  enu_transformation.block<3, 1>(0, 3) = enu_translation;

  Eigen::Matrix4d transformed_pose = gps_src_to_body_.inverse() * enu_transformation;
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  const Eigen::Matrix3d rotation_matrix = gps_src_to_body_.block<3, 3>(0, 0);
  const Eigen::Quaterniond gps_quat(rotation_matrix);

  const Eigen::Quaterniond corrected_orientation = heading_offset_quat_ * gps_quat * imu_orientation;

  pose_measurement->set_header(
      gps_msg->header.stamp.sec,
      gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec / 1e9,
      gps_msg->header.frame_id);

  pose_measurement->set_translation(transformed_translation);
  pose_measurement->set_rotation(corrected_orientation);
  pose_measurement->set_transformation();

  std::lock_guard<std::mutex> lock(*pose_mutex);
  pose_queue->push(pose_measurement);
}

void ROSSubscriber::RosSpin() {
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_);
  executor.spin();
}

}  // namespace ros_wrapper
