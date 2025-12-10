#ifndef ROS_COMMUNICATION_ROS_PUBLISHER_H
#define ROS_COMMUNICATION_ROS_PUBLISHER_H

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using namespace state;

namespace ros_wrapper {

class ROSPublisher {
 public:
  ROSPublisher(std::shared_ptr<rclcpp::Node> node,
               RobotStateQueuePtr& robot_state_queue,
               std::shared_ptr<std::mutex> robot_state_queue_mutex,
               const std::string& config_file);

  ~ROSPublisher();

  void StartPublishingThread();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  RobotStateQueuePtr robot_state_queue_ptr_;
  std::shared_ptr<std::mutex> robot_state_queue_mutex_;

  bool thread_started_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  std::string pose_frame_;
  uint32_t pose_seq_{0};
  double pose_publish_rate_{0.0};
  std::thread pose_publishing_thread_;

  double path_publish_rate_{0.0};
  std::thread path_publishing_thread_;

  std::array<double, 3> first_pose_{{0.0, 0.0, 0.0}};
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::mutex poses_mutex_;
  size_t max_path_length_{1000};

  void PosePublishingThread();
  void PathPublishingThread();

  void PosePublish();

  void TwistPublish(const RobotState& state);
  void AppendPoseForPath(const geometry_msgs::msg::PoseStamped& pose_msg);
  void PathPublish();
};

}  // namespace ros_wrapper

#endif
