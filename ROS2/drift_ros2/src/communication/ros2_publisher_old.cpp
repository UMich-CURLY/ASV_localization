/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.cpp
 *  @author Surya Singh
 *  @brief  Source file for ROS 2 publisher class
 *  @date   March 1, 2025
 **/

 #include "communication/ros2_publisher.h"

 namespace ros_wrapper {
 
 ROSPublisher::ROSPublisher(
     rclcpp::Node::SharedPtr node,
     RobotStateQueuePtr& robot_state_queue_ptr,
     std::shared_ptr<std::mutex> robot_state_queue_mutex,
     bool enable_slip_publisher_)
     : node_(node),
       robot_state_queue_ptr_(robot_state_queue_ptr),
       robot_state_queue_mutex_(robot_state_queue_mutex),
       enable_slip_publisher_(enable_slip_publisher_),
       thread_started_(false) {
  
   std::string pose_topic = "/robot/inekf_estimation/pose";
   std::string path_topic = "/robot/inekf_estimation/path";
   pose_frame_ = "/odom";
   pose_publish_rate_ = 1000; // Hz
   path_publish_rate_ = 10;   // Hz
 
   first_pose_ = {0, 0, 0};
 
   RCLCPP_INFO(node_->get_logger(), "pose_topic: %s, path_topic: %s", pose_topic.c_str(), path_topic.c_str());
   RCLCPP_INFO(node_->get_logger(), "path publish rate: %d", path_publish_rate_);
 
   pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1000);
   path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1000);
 }
 
 ROSPublisher::ROSPublisher(
     rclcpp::Node::SharedPtr node,
     RobotStateQueuePtr& robot_state_queue_ptr,
     std::shared_ptr<std::mutex> robot_state_queue_mutex,
     std::string config_file)
     : node_(node),
       robot_state_queue_ptr_(robot_state_queue_ptr),
       robot_state_queue_mutex_(robot_state_queue_mutex),
       enable_slip_publisher_(enable_slip_publisher_),
       thread_started_(false) {
 
   YAML::Node config = YAML::LoadFile(config_file);
   std::string pose_topic = config["publishers"]["pose_publish_topic"].as<std::string>();
   std::string path_topic = config["publishers"]["path_publish_topic"].as<std::string>();
   pose_frame_ = config["publishers"]["pose_frame"].as<std::string>();
 
   pose_publish_rate_ = config["publishers"]["pose_publish_rate"].as<double>();
   path_publish_rate_ = config["publishers"]["path_publish_rate"].as<double>();
 
   enable_slip_publisher_ = config["publishers"]["enable_slip_publisher"]
                                ? config["publishers"]["enable_slip_publisher"].as<bool>()
                                : false;
 
   first_pose_ = {0, 0, 0};
 
   RCLCPP_INFO(node_->get_logger(), "pose_topic: %s, path_topic: %s", pose_topic.c_str(), path_topic.c_str());
   RCLCPP_INFO(node_->get_logger(), "path publish rate: %d", path_publish_rate_);
 
   pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 1000);
   path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(path_topic, 1000);
 }
 
 ROSPublisher::~ROSPublisher() {
   if (thread_started_) {
     pose_publishing_thread_.join();
     path_publishing_thread_.join();
   }
   poses_.clear();
 }
 
 void ROSPublisher::StartPublishingThread() {
   RCLCPP_INFO(node_->get_logger(), "Starting publishing thread");
 
   pose_publishing_thread_ = std::thread([this] { this->PosePublishingThread(); });
   path_publishing_thread_ = std::thread([this] { this->PathPublishingThread(); });
 
   thread_started_ = true;
 }
 
 void ROSPublisher::PosePublish() {
   if (robot_state_queue_ptr_->empty()) {
     return;
   }
 
   std::lock_guard<std::mutex> lock(*robot_state_queue_mutex_);
   const std::shared_ptr<RobotState> state_ptr = robot_state_queue_ptr_->front();
   robot_state_queue_ptr_->pop();
 
   const RobotState& state = *state_ptr.get();
 
   geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
 
   pose_msg.header.stamp = node_->get_clock()->now();
   pose_msg.header.frame_id = pose_frame_;
 
   pose_msg.pose.pose.position.x = state.get_world_position()(0) - first_pose_[0];
   pose_msg.pose.pose.position.y = state.get_world_position()(1) - first_pose_[1];
   pose_msg.pose.pose.position.z = state.get_world_position()(2) - first_pose_[2];
 
   Eigen::Quaterniond quat(state.get_world_rotation());
   pose_msg.pose.pose.orientation.w = quat.w();
   pose_msg.pose.pose.orientation.x = quat.x();
   pose_msg.pose.pose.orientation.y = quat.y();
   pose_msg.pose.pose.orientation.z = quat.z();
 
   auto& cov = state.get_P();
   for (int i = 0; i < 6; i++) {
     for (int j = 0; j < 6; j++) {
       pose_msg.pose.covariance[i * 6 + j] = cov(i, j);
     }
   }
 
   pose_pub_->publish(pose_msg);
 
   int pose_skip = pose_publish_rate_ / path_publish_rate_;
   if (int(pose_seq_++) % pose_skip == 0) {
     geometry_msgs::msg::PoseStamped pose_stamped;
     pose_stamped.header = pose_msg.header;
     pose_stamped.pose = pose_msg.pose.pose;
 
     std::lock_guard<std::mutex> lock(poses_mutex_);
     poses_.push_back(pose_stamped);
   }
 }
 
 void ROSPublisher::PosePublishingThread() {
   rclcpp::Rate loop_rate(pose_publish_rate_);
   while (rclcpp::ok()) {
     if (enable_slip_publisher_) {
       SlipPublish();
       SlipFlagPublish();
     }
     PosePublish();
     loop_rate.sleep();
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
 
 void ROSPublisher::SlipPublish() {
   // Implementation here
 }
 
 void ROSPublisher::SlipFlagPublish() {
   // Implementation here
 }
 
 }  // namespace ros_wrapper
