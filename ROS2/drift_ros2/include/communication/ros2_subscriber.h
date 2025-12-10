#ifndef ROS_COMMUNICATION_ROS2_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS2_SUBSCRIBER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drift/utils/type_def.h"

using namespace measurement;

using IMUQueuePair = std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>>;
using PositionQueuePair = std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>>;
using PoseQueuePair = std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>>;

namespace ros_wrapper {

class ROSSubscriber {
public:
    ROSSubscriber(rclcpp::Node::SharedPtr node);
    ~ROSSubscriber();

    IMUQueuePair AddIMUSubscriber(const std::string& topic_name);
    PositionQueuePair AddGPS2PositionSubscriber(const std::string& topic_name,
                                                const std::vector<double>& translation_gpssrc2body,
                                                const std::vector<double>& rotation_gpssrc2body);
    PoseQueuePair AddGPSIMU2PoseSubscriber(const std::string& gps_topic_name,
                                           const std::string& imu_topic_name,
                                           const std::vector<double>& translation_gpssrc2body,
                                           const std::vector<double>& rotation_gpssrc2body,
                                           double heading_offset_rad);
    PositionQueuePair AddOdom2PositionSubscriber(const std::string& topic_name,
                                                 const std::vector<double>& translation_odomsrc2body,
                                                 const std::vector<double>& rotation_odomsrc2body);
    void StartSubscribingThread();
    // void StartSubscribingThread(std::shared_ptr<ROSSubscriber> node_ptr);

private:
    void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                     std::shared_ptr<std::mutex> mutex,
                     IMUQueuePtr imu_queue);

    void Odom2PositionCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg,
                               std::shared_ptr<std::mutex> position_mutex,
                               OdomQueuePtr position_queue);

    void GPS2PositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
                              std::shared_ptr<std::mutex> position_mutex,
                              OdomQueuePtr position_queue);
    
    void GPSIMU2PoseCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
                             std::shared_ptr<std::mutex> mutex,
                             const Eigen::Quaterniond& latest_orientation,
                             OdomQueuePtr pose_queue_ptr);

    void RosSpin();

    std::shared_ptr<rclcpp::Node> node_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list_;
    std::vector<IMUQueuePtr> imu_queue_list_;
    std::vector<OdomQueuePtr> position_queue_list_;

    Eigen::Matrix4d odom_src_to_body_{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d gps_src_to_body_{Eigen::Matrix4d::Identity()};

    bool thread_started_{false};
    std::thread subscribing_thread_;

    Eigen::Vector3d reference_position_{Eigen::Vector3d::Zero()};
    bool reference_initialized_{false};

    bool imu_orientation_ready_{false};
    Eigen::Quaterniond heading_offset_quat_{Eigen::Quaterniond::Identity()};
};

} // namespace ros_wrapper

#endif
