#include "communication/ros2_publisher.h"

namespace ros_wrapper {

// ROSPublisher::ROSPublisher(
//     std::shared_ptr<rclcpp::Node> node,
//     RobotStateQueuePtr& robot_state_queue_ptr,
//     std::shared_ptr<std::mutex> robot_state_queue_mutex,
//     bool enable_slip_publisher_)
//     : node_(node),
//       robot_state_queue_ptr_(robot_state_queue_ptr),
//       robot_state_queue_mutex_(robot_state_queue_mutex) {
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
  pose_publish_rate_ = 1000;  // Hz
  path_publish_rate_ = 10;    // Hz
  first_pose_ = {0, 0, 0};

  RCLCPP_INFO(node_->get_logger(), "pose_topic: %s, path_topic: %s", pose_topic_.c_str(), path_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "path publish rate: %f", path_publish_rate_);

  // pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 1000);
  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 1000);
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic_, 1000);

  // pose_timer_ = node_->create_wall_timer(
  //   std::chrono::milliseconds(static_cast<int>(1000.0 / pose_publish_rate_)),
  //     std::bind(&ROSPublisher::PosePublish, this));

  // path_timer_ = node_->create_wall_timer(
  //   std::chrono::milliseconds(static_cast<int>(1000.0 / path_publish_rate_)),
  //     std::bind(&ROSPublisher::PathPublish, this));
}

// ROSPublisher::ROSPublisher(
//   std::shared_ptr<rclcpp::Node> node,
//   RobotStateQueuePtr& robot_state_queue_ptr,
//   std::shared_ptr<std::mutex> robot_state_queue_mutex,
//   const std::string & config_file)
//   : node_(node),
//   robot_state_queue_ptr_(robot_state_queue_ptr),
//   robot_state_queue_mutex_(robot_state_queue_mutex),
//   thread_started_(false) {

//   YAML::Node config = YAML::LoadFile(config_file);
//   std::string pose_topic = config["publishers"]["pose_publish_topic"].as<std::string>();
//   std::string path_topic = config["publishers"]["path_publish_topic"].as<std::string>();
//   pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();

//   pose_publish_rate_ = config["publishers"]["pose_publish_rate"].as<double>();
//   path_publish_rate_ = config["publishers"]["path_publish_rate"].as<double>();

//   enable_slip_publisher_ = config["publishers"]["enable_slip_publisher"]
//       ? config["publishers"]["enable_slip_publisher"].as<bool>()
//       : false;

//   first_pose_ = {0, 0, 0};

//   RCLCPP_INFO(node_->get_logger(), "pose_topic: %s, path_topic: %s", pose_topic.c_str(), path_topic.c_str());
//   RCLCPP_INFO(node_->get_logger(), "path publish rate: %f", path_publish_rate_);

//   pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1000);
//   path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1000);
// }
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
// std::string odom_topic = config["publishers"]["odom_publish_topic"].as<std::string>();

pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();

pose_publish_rate_ = config["publishers"]["pose_publish_rate"].as<double>();
path_publish_rate_ = config["publishers"]["path_publish_rate"].as<double>();

enable_slip_publisher_ = config["publishers"]["enable_slip_publisher"]
       ? config["publishers"]["enable_slip_publisher"].as<bool>()
       : false;

first_pose_ = {0, 0, 0};

RCLCPP_INFO(node_->get_logger(), "Pose topic: %s, Path topic: %s", 
pose_topic.c_str(), path_topic.c_str());

// pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1000);
pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1000);
path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1000);
twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 1000);
// odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000);

prev_state_ = nullptr;
}

// ROSPublisher::~ROSPublisher() {
//   if (thread_started_) {
//     pose_publishing_thread_.join();
//     path_publishing_thread_.join();
//   }
//   poses_.clear();
// }
ROSPublisher::~ROSPublisher() {
  if (thread_started_) {
      if (pose_publishing_thread_.joinable()) pose_publishing_thread_.join();
      if (path_publishing_thread_.joinable()) path_publishing_thread_.join();
  }
  poses_.clear();
}

// void ROSPublisher::StartPublishingThread() {
//   RCLCPP_INFO(node_->get_logger(), "Starting publishing thread");

//   this->pose_publishing_thread_ = std::thread([this] { this->PosePublishingThread(); });
//   this->path_publishing_thread_ = std::thread([this] { this->PathPublishingThread(); });

//   thread_started_ = true;
// }
// void ROSPublisher::StartPublishingThread() {
//   RCLCPP_INFO(node_->get_logger(), "Starting publishing thread");

//   this->pose_publishing_thread_ = std::thread([this] { this->PosePublishingThread(); });
//   this->path_publishing_thread_ = std::thread([this] { this->PathPublishingThread(); });

//   thread_started_ = true;
// }
void ROSPublisher::StartPublishingThread() {
  RCLCPP_INFO(node_->get_logger(), "Starting publishing thread...");
  
  pose_publishing_thread_ = std::thread(&ROSPublisher::PosePublishingThread, this);
  path_publishing_thread_ = std::thread(&ROSPublisher::PathPublishingThread, this);
  
  thread_started_ = true;
}

// void ROSPublisher::PosePublish() {
//   if (robot_state_queue_ptr_->empty()) {
//     RCLCPP_WARN(node_->get_logger(), "Robot state queue is empty!");
//     return;
//   }

//   // auto start = std::chrono::high_resolution_clock::now();
//   // std::lock_guard<std::mutex> lock(*robot_state_queue_mutex_);
//   // auto end = std::chrono::high_resolution_clock::now();

//   // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//   // RCLCPP_INFO(node_->get_logger(), "Mutex locked for: %ld microseconds", duration.count());

//   // Get the first pose
//   robot_state_queue_mutex_.get()->lock();
//   auto state_ptr = robot_state_queue_ptr_->front();
//   robot_state_queue_ptr_->pop();
//   robot_state_queue_mutex_.get()->unlock();

//   const RobotState& state = *state_ptr.get();
//   geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

//   // pose_msg.header.stamp = node_->get_clock()->now();
//   pose_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
//   pose_msg.header.frame_id = pose_frame_;
//   pose_msg.pose.pose.position.x = state.get_world_position()(0) - first_pose_[0];
//   pose_msg.pose.pose.position.y = state.get_world_position()(1) - first_pose_[1];
//   pose_msg.pose.pose.position.z = state.get_world_position()(2) - first_pose_[2];

//   Eigen::Quaterniond quat(state.get_world_rotation());
//   pose_msg.pose.pose.orientation.w = quat.w();
//   pose_msg.pose.pose.orientation.x = quat.x();
//   pose_msg.pose.pose.orientation.y = quat.y();
//   pose_msg.pose.pose.orientation.z = quat.z();

//   auto& cov = state.get_P();
//   for (int i = 0; i < 6; i++) {
//     for (int j = 0; j < 6; j++) {
//       pose_msg.pose.covariance[i * 6 + j] = cov(i, j);
//     }
//   }

//   pose_pub_->publish(pose_msg);
//   pose_seq_++;

//   int pose_skip = pose_publish_rate_
//                   / path_publish_rate_;    // Pose publish rate must be faster
//                                            // than path publish rate

//   // if (pose_seq_ % static_cast<uint32_t>(pose_publish_rate_ / path_publish_rate_) == 0) {
//     if (int(pose_seq_) % pose_skip == 0) {
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     pose_stamped.header = pose_msg.header;
//     pose_stamped.pose = pose_msg.pose.pose;

//     // std::lock_guard<std::mutex> poses_lock(poses_mutex_);
//     std::lock_guard<std::mutex> lock(poses_mutex_);
//     poses_.push_back(pose_stamped);
//   }
  
// }

// void ROSPublisher::PosePublish() {
//   if (robot_state_queue_ptr_->empty()) {
//     // RCLCPP_WARN(node_->get_logger(), "Robot state queue is empty!");
//     return;
// }

//   std::lock_guard<std::mutex> lock(*robot_state_queue_mutex_);
//   const std::shared_ptr<RobotState> state_ptr = robot_state_queue_ptr_->front();
//   robot_state_queue_ptr_->pop();

//   const RobotState& state = *state_ptr.get();
//   geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

//   pose_msg.header.stamp = node_->get_clock()->now();  // ROS2: Use node's clock
//   pose_msg.header.frame_id = pose_frame_;

//   pose_msg.pose.pose.position.x = state.get_world_position()(0) - first_pose_[0];
//   pose_msg.pose.pose.position.y = state.get_world_position()(1) - first_pose_[1];
//   pose_msg.pose.pose.position.z = state.get_world_position()(2) - first_pose_[2];

//   Eigen::Quaterniond quat(state.get_world_rotation());
//   pose_msg.pose.pose.orientation.w = quat.w();
//   pose_msg.pose.pose.orientation.x = quat.x();
//   pose_msg.pose.pose.orientation.y = quat.y();
//   pose_msg.pose.pose.orientation.z = quat.z();

//   pose_pub_->publish(pose_msg);

//   pose_seq_++;

//   int pose_skip = pose_publish_rate_
//                   / path_publish_rate_;    // Pose publish rate must be faster
//                                            // than path publish rate

//   // if (pose_seq_ % static_cast<uint32_t>(pose_publish_rate_ / path_publish_rate_) == 0) {
//   if (int(pose_seq_) % pose_skip == 0) {
//     geometry_msgs::msg::PoseStamped pose_stamped;
//     pose_stamped.header = pose_msg.header;
//     pose_stamped.pose = pose_msg.pose.pose;

//     // std::lock_guard<std::mutex> poses_lock(poses_mutex_);
//     std::lock_guard<std::mutex> lock(poses_mutex_);
//     poses_.push_back(pose_stamped);
//   }
// }
void ROSPublisher::PosePublish() {
  auto start = std::chrono::steady_clock::now();
  if (robot_state_queue_ptr_->empty()) {
      // RCLCPP_WARN(node_->get_logger(), "Robot state queue is empty!");
      return;
  }
  // RCLCPP_INFO(node_->get_logger(), "[PosePublish] Queue size: %ld", robot_state_queue_ptr_->size());

  // auto lock_start = std::chrono::steady_clock::now();
  // Get the first pose
  std::lock_guard<std::mutex> lock(*robot_state_queue_mutex_);
  // auto lock_end = std::chrono::steady_clock::now();
  // auto lock_wait = std::chrono::duration_cast<std::chrono::microseconds>(lock_end - lock_start).count();
  // RCLCPP_INFO(node_->get_logger(), "[PosePublish] Mutex lock wait (us): %ld", lock_wait);
  
  const std::shared_ptr<RobotState> state_ptr = robot_state_queue_ptr_->front();
  robot_state_queue_ptr_->pop();
  
  // static int pose_pub_counter = 0;
  // static auto last_print = std::chrono::steady_clock::now();
  // pose_pub_counter++;

  // auto now = std::chrono::steady_clock::now();
  // if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1) {
  //     RCLCPP_INFO(node_->get_logger(), "[PosePublish] Calls/sec: %d", pose_pub_counter);
  //     pose_pub_counter = 0;
  //     last_print = now;
  // }

  const RobotState& state = *state_ptr.get();

  // geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  geometry_msgs::msg::PoseStamped pose_msg;

  // Header
  // pose_msg.header.stamp = node_->get_clock()->now();
  pose_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  pose_msg.header.frame_id = pose_frame_;
  // pose_msg.header.frame_id = pose_frame_;

  // Pose
  // pose_msg.pose.pose.position.x = state.get_world_position()(0) - first_pose_[0];
  // pose_msg.pose.pose.position.y = state.get_world_position()(1) - first_pose_[1];
  // pose_msg.pose.pose.position.z = state.get_world_position()(2) - first_pose_[2];
  pose_msg.pose.position.x = state.get_world_position()(0) - first_pose_[0];
  pose_msg.pose.position.y = state.get_world_position()(1) - first_pose_[1];
  pose_msg.pose.position.z = state.get_world_position()(2) - first_pose_[2];

  Eigen::Quaterniond quat(state.get_world_rotation());
  // pose_msg.pose.pose.orientation.w = quat.w();
  // pose_msg.pose.pose.orientation.x = quat.x();
  // pose_msg.pose.pose.orientation.y = quat.y();
  // pose_msg.pose.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  // Covariance
  // auto& cov = state.get_P();
  // for (int i = 0; i < 6; i++) {
  //     for (int j = 0; j < 6; j++) {
  //         pose_msg.pose.covariance[i * 6 + j] = cov(i, j);
  //     }
  // }

  pose_pub_->publish(pose_msg);

  pose_seq_++;
  TwistPublish(state);
  // OdometryPublish(pose_msg, state); 


  // int pose_skip = pose_publish_rate_ / path_publish_rate_;
  int pose_skip = std::max(1, static_cast<int>(pose_publish_rate_ / path_publish_rate_));

  // std::cout << "pose_skip: " << pose_skip << std::endl;
  // std::cout << "int(pose_seq_): " << int(pose_seq_) << std::endl;
  if (int(pose_seq_) % pose_skip == 0) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = pose_msg.header;
      // pose_stamped.pose = pose_msg.pose.pose;
      pose_stamped.pose = pose_msg.pose;

      std::lock_guard<std::mutex> poses_lock(poses_mutex_);
      poses_.push_back(pose_stamped);

      // Trim old poses if we exceed the max length
      if (poses_.size() > max_path_length_) {
        poses_.erase(poses_.begin(), poses_.begin() + (poses_.size() - max_path_length_));
      }
  }

  // auto end = std::chrono::steady_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  // RCLCPP_INFO(node_->get_logger(), "[PosePublish] Duration (us): %ld", duration);
}

void ROSPublisher::TwistPublish(const RobotState& state) {
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(state.get_time() * 1e9));
  twist_msg.header.frame_id = pose_frame_;  // Same as pose frame

  // Linear velocity
  Eigen::Vector3d lin_vel = state.get_world_velocity();  // This is the linear velocity

  twist_msg.twist.linear.x = lin_vel.x();
  twist_msg.twist.linear.y = lin_vel.y();
  twist_msg.twist.linear.z = lin_vel.z();

  // Angular velocity estimation (from rotation difference)
  // if (prev_state_) {
  //     double dt = state.get_time() - prev_state_->get_time();
  //     if (dt > 1e-6) {
  //         Eigen::Quaterniond q1(prev_state_->get_world_rotation());
  //         Eigen::Quaterniond q2(state.get_world_rotation());

  //         Eigen::Quaterniond dq = q2 * q1.inverse();
  //         Eigen::AngleAxisd angle_axis(dq);

  //         Eigen::Vector3d ang_vel = angle_axis.axis() * angle_axis.angle() / dt;

  //         twist_msg.twist.angular.x = ang_vel.x();
  //         twist_msg.twist.angular.y = ang_vel.y();
  //         twist_msg.twist.angular.z = ang_vel.z();
  //     }
  // }

  twist_pub_->publish(twist_msg);

  // Save current state for next angular velocity computation
  prev_state_ = std::make_shared<RobotState>(state);
}

// void ROSPublisher::OdometryPublish(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg, const RobotState& state) {
//   nav_msgs::msg::Odometry odom_msg;

//   // Copy header and pose from existing pose_msg
//   odom_msg.header = pose_msg.header;
//   odom_msg.pose = pose_msg.pose;
//   odom_msg.child_frame_id = pose_frame_;  // Typically "base_link"

//   // Linear velocity
//   Eigen::Vector3d lin_vel = state.get_world_velocity();
//   odom_msg.twist.twist.linear.x = lin_vel.x();
//   odom_msg.twist.twist.linear.y = lin_vel.y();
//   odom_msg.twist.twist.linear.z = lin_vel.z();

//   // Angular velocity estimation
//   if (prev_state_) {
//       double dt = state.get_time() - prev_state_->get_time();
//       if (dt > 1e-6) {
//           Eigen::Quaterniond q1(prev_state_->get_world_rotation());
//           Eigen::Quaterniond q2(state.get_world_rotation());

//           Eigen::Quaterniond dq = q2 * q1.inverse();
//           Eigen::AngleAxisd angle_axis(dq);
//           Eigen::Vector3d ang_vel = angle_axis.axis() * angle_axis.angle() / dt;

//           odom_msg.twist.twist.angular.x = ang_vel.x();
//           odom_msg.twist.twist.angular.y = ang_vel.y();
//           odom_msg.twist.twist.angular.z = ang_vel.z();
//       }
//   }

//   odom_pub_->publish(odom_msg);
//   prev_state_ = std::make_shared<RobotState>(state);
// }



// Pose publishing thread
// void ROSPublisher::PosePublishingThread() {
//   rclcpp::Rate loop_rate(pose_publish_rate_);
//   while (rclcpp::ok()) {
//     if (enable_slip_publisher_) {
//       SlipPublish();
//       SlipFlagPublish();
//     }
//     PosePublish();
//     loop_rate.sleep();
//   }
// }
// void ROSPublisher::PosePublishingThread() {
//   rclcpp::Rate loop_rate(pose_publish_rate_);
//   while (rclcpp::ok()) {
//       PosePublish();
//       loop_rate.sleep();
//   }
// }
void ROSPublisher::PosePublishingThread() {
  rclcpp::Rate loop_rate(pose_publish_rate_);

  while (rclcpp::ok()) {
      // auto loop_start = std::chrono::steady_clock::now();
      if (enable_slip_publisher_) {
          SlipPublish();
          SlipFlagPublish();
      }
      PosePublish();

      // auto loop_end = std::chrono::steady_clock::now();
      // auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start).count();
      // RCLCPP_INFO(node_->get_logger(), "[PosePublishingThread] Loop duration (us): %ld", loop_duration);
      loop_rate.sleep();
  }
}

// void ROSPublisher::PathPublish() {
//   std::lock_guard<std::mutex> lock(poses_mutex_);
//   if (poses_.empty()) {
//     return;
//   }

//   nav_msgs::msg::Path path_msg;
//   // path_msg.header.stamp = node_->get_clock()->now();
//   path_msg.header.stamp = poses_.back().header.stamp;
//   path_msg.header.frame_id = pose_frame_;
//   path_msg.poses = poses_;

//   path_pub_->publish(path_msg);
//   path_seq_++;
// }
// void ROSPublisher::PathPublish() {
//   std::lock_guard<std::mutex> lock(poses_mutex_);
//   if (poses_.empty()) {
//       return;
//   }

//   nav_msgs::msg::Path path_msg;
//   path_msg.header.stamp = node_->get_clock()->now();
//   path_msg.header.frame_id = pose_frame_;
//   path_msg.poses = poses_;

//   path_pub_->publish(path_msg);
//   path_seq_++;
// }
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
  // auto end = std::chrono::steady_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  // RCLCPP_INFO(node_->get_logger(), "[PathPublish] Duration (us): %ld", duration);
}

// Path publishing thread
// void ROSPublisher::PathPublishingThread() {
//   rclcpp::Rate loop_rate(path_publish_rate_);
//   while (rclcpp::ok()) {
//     PathPublish();
//     loop_rate.sleep();
//   }
// }
// void ROSPublisher::PathPublishingThread() {
//   rclcpp::Rate loop_rate(path_publish_rate_);
//   while (rclcpp::ok()) {
//       PathPublish();
//       loop_rate.sleep();
//   }
// }
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
