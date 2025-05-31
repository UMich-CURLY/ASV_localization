/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_publisher.h
 *  @author Surya Singh
 *  @brief  Header file for ROS publisher class
 *  @date   March 1, 2025
 **/

#ifndef ROS_COMMUNICATION_ROS_PUBLISHER_H
#define ROS_COMMUNICATION_ROS_PUBLISHER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "boost/bind/bind.hpp"
#include <nav_msgs/msg/path.hpp>
#include "yaml-cpp/yaml.h"

#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using namespace state;

namespace ros_wrapper {

/**
 * @class ROSPublisher
 * @brief ROS 2 publisher class for publishing robot state to ROS topics
 */
class ROSPublisher {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Construct a new ROS Publisher object
   *
   * @param[in] robot_state_queue: Robot state queue
   * @param[in] robot_state_queue_mutex: Robot state queue mutex
   * @param[in] enable_slip_publisher: Flag for slip publisher
   */
  ROSPublisher(rclcpp::Node::SharedPtr node, RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex,
               bool enable_slip_publisher = false);

  /**
   * @brief Construct a new ROS Publisher object with configuration file
   *
   * @param[in] robot_state_queue: Robot state queue
   * @param[in] robot_state_queue_mutex: Robot state queue mutex
   * @param[in] config_file: Configuration file
   */
  ROSPublisher(rclcpp::Node::SharedPtr node, RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex,
               std::string config_file);

  /// @}

  /// @name Destructors
  /// @{
  /**
   * @brief Destroy the ROSPublisher object
   */
  ~ROSPublisher();
  /// @}

  /**
   * @brief Start publishing thread for pose and path publishers
   */
  void StartPublishingThread();

 private:
  // std::shared_ptr<rclcpp::Node> node_;  // ROS 2 Node
  rclcpp::Node::SharedPtr node_;

  RobotStateQueuePtr robot_state_queue_ptr_;  // Pointer to the robot state queue
  std::shared_ptr<std::mutex> robot_state_queue_mutex_;  // Mutex for the state queue

  bool thread_started_;  // Flag for thread started

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;  
  std::string pose_frame_;  // Frame name for pose
  uint32_t pose_seq_ = 0;   // Sequence number for pose publisher
  double pose_publish_rate_;  
  std::thread pose_publishing_thread_;  // Pose publishing thread

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  
  uint32_t path_seq_ = 0;  // Sequence number for path publisher
  double path_publish_rate_;
  std::thread path_publishing_thread_;  // Path publishing thread

  int pose_skip_;  // Number of poses to skip while generating a path message

  std::array<float, 3> first_pose_;  // Initial pose of the robot
  std::vector<geometry_msgs::msg::PoseStamped> poses_;  // Path message
  std::mutex poses_mutex_;  // Mutex for path storage

  bool enable_slip_publisher_;  // Flag for slip publisher

  /// @name Publishing methods
  /// @{
  /**
   * @brief A thread for publishing path messages
   */
  void PathPublishingThread();

  /**
   * @brief Publish the path message
   */
  void PathPublish();

  /**
   * @brief A thread for publishing pose messages
   */
  void PosePublishingThread();

  /**
   * @brief Publish a pose message
   */
  void PosePublish();

  /**
   * @brief Publish slip velocity
   */
  void SlipPublish();

  /**
   * @brief Publish slip flag
   */
  void SlipFlagPublish();
  /// @}
};

}  // namespace ros_wrapper

#endif  // ROS_COMMUNICATION_ROS_PUBLISHER_H
