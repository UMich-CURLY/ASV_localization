#ifndef ROS_COMMUNICATION_ROS_PUBLISHER_H
#define ROS_COMMUNICATION_ROS_PUBLISHER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <thread>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "drift/state/robot_state.h"
#include "drift/utils/type_def.h"

using namespace state;

namespace ros_wrapper {

class ROSPublisher {
 public:
  // ROSPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // ROSPublisher(std::shared_ptr<rclcpp::Node> node,
  //              RobotStateQueuePtr & robot_state_queue,
  //              std::shared_ptr<std::mutex> robot_state_queue_mutex,
  //              bool enable_slip_publisher = false);
  ROSPublisher(std::shared_ptr<rclcpp::Node> node,
    RobotStateQueuePtr & robot_state_queue,
    std::shared_ptr<std::mutex> robot_state_queue_mutex,
    bool enable_slip_publisher = false);

  ROSPublisher(std::shared_ptr<rclcpp::Node> node,
    RobotStateQueuePtr & robot_state_queue,
    std::shared_ptr<std::mutex> robot_state_queue_mutex,
    const std::string & config_file);

  ~ROSPublisher();

  void StartPublishingThread();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  RobotStateQueuePtr robot_state_queue_ptr_;
  std::shared_ptr<std::mutex> robot_state_queue_mutex_;

  bool thread_started_;            // Flag for thread started

  // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_alias_pubs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_alias_pubs_;
  // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;

  std::string pose_frame_;
  std::string twist_frame_;
  uint32_t pose_seq_ = 0;
  double pose_publish_rate_;
  std::thread pose_publishing_thread_;    // Thread for pose publishing

  double path_publish_rate_;
  uint32_t path_seq_ = 0;
  std::thread path_publishing_thread_;    // Thread for path publishing

  int pose_skip_;

  std::array<float, 3> first_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::mutex poses_mutex_;
  size_t max_path_length_ = 1000;
  bool enable_slip_publisher_;

  std::shared_ptr<RobotState> prev_state_;


  void PathPublishingThread();
  void PathPublish();

  void PosePublishingThread();
  void PosePublish();

  void TwistPublish(const RobotState& state);
  // void OdometryPublish(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg,
  //   const RobotState& state);


  void SlipPublish();
  void SlipFlagPublish();
};

}  // namespace ros_wrapper

#endif
