#include "communication/ros2_publisher.h"

#include <Eigen/Geometry>

namespace ros_wrapper {

ROSPublisher::ROSPublisher(std::shared_ptr<rclcpp::Node> node,
                             RobotStateQueuePtr& robot_state_queue_ptr,
                             std::shared_ptr<std::mutex> robot_state_queue_mutex,
                             const std::string& config_file)
    : node_(node),
      robot_state_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_mutex_(robot_state_queue_mutex) {
  YAML::Node config = YAML::LoadFile(config_file);

  const std::string pose_topic = config["publishers"]["pose_publish_topic"].as<std::string>();
  const std::string path_topic = config["publishers"]["path_publish_topic"].as<std::string>();
  const std::string twist_topic = config["publishers"]["twist_publish_topic"].as<std::string>();

  pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();
  pose_publish_rate_ = std::max(1.0, config["publishers"]["pose_publish_rate"].as<double>());
  path_publish_rate_ = std::max(1.0, config["publishers"]["path_publish_rate"].as<double>());

  RCLCPP_INFO(node_->get_logger(), "Publishing pose on %s and path on %s",
              pose_topic.c_str(), path_topic.c_str());

  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1000);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1000);
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 1000);
}

ROSPublisher::~ROSPublisher() {
  if (thread_started_) {
    if (pose_publishing_thread_.joinable()) pose_publishing_thread_.join();
    if (path_publishing_thread_.joinable()) path_publishing_thread_.join();
  }
  poses_.clear();
}

void ROSPublisher::StartPublishingThread() {
  if (thread_started_) {
    RCLCPP_WARN(node_->get_logger(), "Publishing threads already started");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Starting publishing thread...");

  pose_publishing_thread_ = std::thread(&ROSPublisher::PosePublishingThread, this);
  path_publishing_thread_ = std::thread(&ROSPublisher::PathPublishingThread, this);

  thread_started_ = true;
}

void ROSPublisher::PosePublish() {
  std::shared_ptr<RobotState> state_ptr;
  {
    std::lock_guard<std::mutex> lock(*robot_state_queue_mutex_);
    if (robot_state_queue_ptr_->empty()) {
      return;
    }
    state_ptr = robot_state_queue_ptr_->front();
    robot_state_queue_ptr_->pop();
  }

  const RobotState& state = *state_ptr;

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  pose_msg.header.frame_id = pose_frame_;

  pose_msg.pose.position.x = state.get_world_position()(0) - first_pose_[0];
  pose_msg.pose.position.y = state.get_world_position()(1) - first_pose_[1];
  pose_msg.pose.position.z = state.get_world_position()(2) - first_pose_[2];

  const Eigen::Quaterniond quat(state.get_world_rotation());
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  pose_pub_->publish(pose_msg);

  ++pose_seq_;
  TwistPublish(state);
  AppendPoseForPath(pose_msg);
}

void ROSPublisher::TwistPublish(const RobotState& state) {
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  twist_msg.header.frame_id = pose_frame_;  // Same as pose frame

  const Eigen::Vector3d lin_vel = state.get_world_velocity();

  twist_msg.twist.linear.x = lin_vel.x();
  twist_msg.twist.linear.y = lin_vel.y();
  twist_msg.twist.linear.z = lin_vel.z();

  const Eigen::Vector3d ang_vel = state.get_body_angular_velocity();
  twist_msg.twist.angular.x = ang_vel.x();
  twist_msg.twist.angular.y = ang_vel.y();
  twist_msg.twist.angular.z = ang_vel.z();

  twist_pub_->publish(twist_msg);
}

void ROSPublisher::PosePublishingThread() {
  rclcpp::Rate loop_rate(pose_publish_rate_);

  while (rclcpp::ok()) {
    PosePublish();
    loop_rate.sleep();
  }
}

void ROSPublisher::AppendPoseForPath(const geometry_msgs::msg::PoseStamped& pose_msg) {
  if (path_publish_rate_ <= 0.0) {
    return;
  }

  const int pose_skip = std::max(1, static_cast<int>(pose_publish_rate_ / path_publish_rate_));

  if (pose_seq_ % pose_skip != 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(poses_mutex_);
  poses_.push_back(pose_msg);
  if (poses_.size() > max_path_length_) {
    poses_.erase(poses_.begin(), poses_.begin() + (poses_.size() - max_path_length_));
  }
}

void ROSPublisher::PathPublish() {
  std::lock_guard<std::mutex> lock(poses_mutex_);

  if (poses_.empty()) {
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = poses_.back().header.stamp;
  path_msg.header.frame_id = pose_frame_;
  path_msg.poses = poses_;

  path_pub_->publish(path_msg);
}

void ROSPublisher::PathPublishingThread() {
  rclcpp::Rate loop_rate(path_publish_rate_);

  while (rclcpp::ok()) {
    PathPublish();
    loop_rate.sleep();
  }
}

}  // namespace ros_wrapper
