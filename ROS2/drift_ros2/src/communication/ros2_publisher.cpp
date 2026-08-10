#include "communication/ros2_publisher.h"

namespace ros_wrapper {

namespace {

std::vector<std::string> ReadTopicList(const YAML::Node& parent, const std::string& key) {
  std::vector<std::string> topics;
  const YAML::Node node = parent[key];
  if (!node) {
    return topics;
  }
  if (node.IsSequence()) {
    for (const auto& item : node) {
      topics.push_back(item.as<std::string>());
    }
  } else {
    topics.push_back(node.as<std::string>());
  }
  return topics;
}

rclcpp::QoS LatestValueQoS() {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

}  // namespace

ROSPublisher::ROSPublisher(
    std::shared_ptr<rclcpp::Node> node,
    RobotStateQueuePtr& robot_state_queue_ptr,
    std::shared_ptr<std::mutex> robot_state_queue_mutex,
    bool enable_slip_publisher_)
    : node_(node),  // Correctly initialize the base class
      robot_state_queue_ptr_(robot_state_queue_ptr),
      robot_state_queue_mutex_(robot_state_queue_mutex),
      thread_started_(false) {
  std::string pose_topic_ = "/robot/inekf_estimation/pose";
  std::string path_topic_ = "/robot/inekf_estimation/path";
  pose_frame_ = "odom";
  twist_frame_ = "base_link";
  pose_publish_rate_ = 20;  // Hz
  path_publish_rate_ = 10;    // Hz
  first_pose_ = {0, 0, 0};

  RCLCPP_INFO(node_->get_logger(), "pose_topic: %s, path_topic: %s", pose_topic_.c_str(), path_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "path publish rate: %f", path_publish_rate_);

  const auto latest_value_qos = LatestValueQoS();
  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic_, latest_value_qos);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      path_topic_, latest_value_qos);

}


ROSPublisher::ROSPublisher(std::shared_ptr<rclcpp::Node> node,
  RobotStateQueuePtr& robot_state_queue_ptr,
  std::shared_ptr<std::mutex> robot_state_queue_mutex,
  const std::string& config_file)
: node_(node),
robot_state_queue_ptr_(robot_state_queue_ptr),
robot_state_queue_mutex_(robot_state_queue_mutex),
thread_started_(false) {

YAML::Node config = YAML::LoadFile(config_file);

std::string pose_topic = config["publishers"]["pose_publish_topic"].as<std::string>();
std::string path_topic = config["publishers"]["path_publish_topic"].as<std::string>();
std::string twist_topic = config["publishers"]["twist_publish_topic"].as<std::string>();
std::vector<std::string> pose_alias_topics =
    ReadTopicList(config["publishers"], "pose_publish_topic_aliases");
std::vector<std::string> twist_alias_topics =
    ReadTopicList(config["publishers"], "twist_publish_topic_aliases");

pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();
twist_frame_ = config["publishers"]["twist_frame"]
    ? config["publishers"]["twist_frame"].as<std::string>()
    : pose_frame_;

pose_publish_rate_ = config["publishers"]["pose_publish_rate"].as<double>();
path_publish_rate_ = config["publishers"]["path_publish_rate"].as<double>();

enable_slip_publisher_ = config["publishers"]["enable_slip_publisher"]
       ? config["publishers"]["enable_slip_publisher"].as<bool>()
       : false;

first_pose_ = {0, 0, 0};

RCLCPP_INFO(node_->get_logger(), "Pose topic: %s, Path topic: %s", 
pose_topic.c_str(), path_topic.c_str());

const auto latest_value_qos = LatestValueQoS();
pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    pose_topic, latest_value_qos);
path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    path_topic, latest_value_qos);
twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    twist_topic, latest_value_qos);
for (const auto& alias_topic : pose_alias_topics) {
  pose_alias_pubs_.push_back(
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          alias_topic, latest_value_qos));
  RCLCPP_INFO(node_->get_logger(), "Pose compatibility alias: %s", alias_topic.c_str());
}
for (const auto& alias_topic : twist_alias_topics) {
  twist_alias_pubs_.push_back(
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          alias_topic, latest_value_qos));
  RCLCPP_INFO(node_->get_logger(), "Twist compatibility alias: %s", alias_topic.c_str());
}

prev_state_ = nullptr;
}

ROSPublisher::~ROSPublisher() {
  if (thread_started_) {
      if (pose_publishing_thread_.joinable()) pose_publishing_thread_.join();
      if (path_publishing_thread_.joinable()) path_publishing_thread_.join();
  }
  poses_.clear();
}

void ROSPublisher::StartPublishingThread() {
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
  
  while (robot_state_queue_ptr_->size() > 1) {
    robot_state_queue_ptr_->pop();
  }
  state_ptr = robot_state_queue_ptr_->front();
    robot_state_queue_ptr_->pop();
  }

  const RobotState& state = *state_ptr.get();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  pose_msg.header.frame_id = pose_frame_;


  // Pose
  pose_msg.pose.position.x = state.get_world_position()(0) - first_pose_[0];
  pose_msg.pose.position.y = state.get_world_position()(1) - first_pose_[1];
  pose_msg.pose.position.z = state.get_world_position()(2) - first_pose_[2];

  Eigen::Quaterniond quat(state.get_world_rotation());
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();


  pose_pub_->publish(pose_msg);
  for (const auto& alias_pub : pose_alias_pubs_) {
    alias_pub->publish(pose_msg);
  }

  pose_seq_++;
  TwistPublish(state);

  int pose_skip = std::max(1, static_cast<int>(pose_publish_rate_ / path_publish_rate_));

  if (int(pose_seq_) % pose_skip == 0) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = pose_msg.header;
      pose_stamped.pose = pose_msg.pose;

      std::lock_guard<std::mutex> poses_lock(poses_mutex_);
      poses_.push_back(pose_stamped);

      // Trim old poses if we exceed the max length
      if (poses_.size() > max_path_length_) {
        poses_.erase(poses_.begin(), poses_.begin() + (poses_.size() - max_path_length_));
      }
  }
}

void ROSPublisher::TwistPublish(const RobotState& state) {
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  twist_msg.header.frame_id = twist_frame_;

  // Publish body-frame twist so it is directly comparable to Odometry.child_frame_id semantics.
  const Eigen::Vector3d lin_vel = state.get_body_velocity();
  const Eigen::Vector3d ang_vel = state.get_body_angular_velocity();

  twist_msg.twist.linear.x = lin_vel.x();
  twist_msg.twist.linear.y = lin_vel.y();
  twist_msg.twist.linear.z = lin_vel.z();
  twist_msg.twist.angular.x = ang_vel.x();
  twist_msg.twist.angular.y = ang_vel.y();
  twist_msg.twist.angular.z = ang_vel.z();

  twist_pub_->publish(twist_msg);
  for (const auto& alias_pub : twist_alias_pubs_) {
    alias_pub->publish(twist_msg);
  }

  // Save current state for next angular velocity computation
  prev_state_ = std::make_shared<RobotState>(state);
}

void ROSPublisher::PosePublishingThread() {
  rclcpp::Rate loop_rate(pose_publish_rate_);

  while (rclcpp::ok()) {
      // auto loop_start = std::chrono::steady_clock::now();
      if (enable_slip_publisher_) {
          SlipPublish();
          SlipFlagPublish();
      }
      PosePublish();

      loop_rate.sleep();
  }
}

void ROSPublisher::PathPublish() {
  // auto start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(poses_mutex_);

  if (poses_.empty()) {
      return;
  }

  nav_msgs::msg::Path path_msg;
  // path_msg.header.stamp = node_->get_clock()->now();
  path_msg.header.stamp = poses_.back().header.stamp;
  path_msg.header.frame_id = pose_frame_;
  path_msg.poses = poses_;

  path_pub_->publish(path_msg);
  path_seq_++;
}

void ROSPublisher::PathPublishingThread() {
  rclcpp::Rate loop_rate(path_publish_rate_);

  while (rclcpp::ok()) {
      PathPublish();
      loop_rate.sleep();
  }
}

void ROSPublisher::SlipPublish() {
  
}

void ROSPublisher::SlipFlagPublish() {
  // Get state
  
}

} // namespace ros_wrapper
