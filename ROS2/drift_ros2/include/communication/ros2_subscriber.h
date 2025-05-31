#ifndef ROS_COMMUNICATION_ROS2_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS2_SUBSCRIBER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "drift/estimator/inekf_estimator.h"
//#include "drift/kinematics/mini_cheetah_kinematics.h"
#include "drift/utils/type_def.h"

using namespace measurement;

typedef std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> IMUQueuePair;
typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> PositionQueuePair;
typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> OdomQueuePair;
typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> PoseQueuePair;

typedef message_filters::Subscriber<sensor_msgs::msg::Imu> IMUMsgFilterT;
typedef message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> IMUOffsetMsgFilterT;
typedef std::shared_ptr<IMUMsgFilterT> IMUMsgFilterTPtr;
typedef std::shared_ptr<IMUOffsetMsgFilterT> IMUOffsetMsgFilterTPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::Vector3Stamped> IMUSyncPolicy;
typedef std::shared_ptr<message_filters::Synchronizer<IMUSyncPolicy>> IMUSyncPtr;

typedef std::queue<std::shared_ptr<NavSatMeasurement<double>>> GPSNavSatQueue;
typedef std::shared_ptr<GPSNavSatQueue> GPSNavSatQueuePtr;
typedef std::pair<GPSNavSatQueuePtr, std::shared_ptr<std::mutex>> GPSNavSatQueuePair;

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
                                           const std::vector<double>& rotation_gpssrc2body);
    PositionQueuePair AddOdom2PositionSubscriber(const std::string& topic_name,
                                                 const std::vector<double>& translation_odomsrc2body,
                                                 const std::vector<double>& rotation_odomsrc2body);
    void StartSubscribingThread();
    // void StartSubscribingThread(std::shared_ptr<ROSSubscriber> node_ptr);

private:
    // void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg,
    //                   const std::shared_ptr<std::mutex>& mutex,
    //                   IMUQueuePtr& imu_queue);
    // void IMUCallback(
    //     const sensor_msgs::msg::Imu::SharedPtr imu_msg,
    //     const std::shared_ptr<std::mutex>& mutex, const IMUQueuePtr& imu_queue);
    
    void IMUCallback(
            const sensor_msgs::msg::Imu::SharedPtr imu_msg, 
            std::shared_ptr<std::mutex> mutex, 
            IMUQueuePtr imu_queue);

    // void Odom2PositionCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    //                             const std::shared_ptr<std::mutex>& position_mutex,
    //                             OdomQueuePtr& position_queue);
    void Odom2PositionCallback(
        const nav_msgs::msg::Odometry::SharedPtr odom_msg,
        std::shared_ptr<std::mutex> position_mutex, OdomQueuePtr position_queue);

    // void GPS2PositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    //                            const std::shared_ptr<std::mutex>& position_mutex,
    //                            OdomQueuePtr& position_queue,
    //                            const Eigen::Vector3d& reference_position);
    // void GPS2PositionCallback(
    //     const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    //     std::shared_ptr<std::mutex> position_mutex, OdomQueuePtr position_queue, const Eigen::Vector3d& reference_position);
    void GPS2PositionCallback(
        const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
        std::shared_ptr<std::mutex> position_mutex, OdomQueuePtr position_queue, 
        Eigen::Vector3d& reference_position);
    
    void GPSIMU2PoseCallback(
        const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
        std::shared_ptr<std::mutex> mutex, const Eigen::Quaterniond& latest_orientation, OdomQueuePtr pose_queue_ptr,
        Eigen::Vector3d& reference_position);

    void RosSpin();
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list_;

    // measurement queue list
    std::vector<IMUQueuePtr> imu_queue_list_;    // List of IMU queue pointers
  
    std::vector<OdomQueuePtr>
      position_queue_list_;    // List of pose queue pointers

    std::vector<OdomQueuePtr> pose_queue_list_;

    std::vector<IMUSyncPtr> imu_sync_list_;
    std::vector<std::shared_ptr<std::mutex>> mutex_list_;
    std::unordered_map<int, OdomMeasurementPtr> prev_odom_map_;
    Eigen::Matrix4d odom_src_to_body_;
    Eigen::Matrix4d gps_src_to_body_;

    bool thread_started_;
    std::thread subscribing_thread_;

    // Globals
    Eigen::Vector3d reference_position;
    bool reference_initialized = false;

    std::shared_ptr<Eigen::Quaterniond> initial_orientation = std::make_shared<Eigen::Quaterniond>();
    bool initial_orientation_set = false;



    rclcpp::executors::MultiThreadedExecutor executor;

    int odom_src_id_ = 0;    // Keep track of the odom source id, start from 0 and
                           // increment by 1 for each new odom source
};

} // namespace ros_wrapper

#endif

