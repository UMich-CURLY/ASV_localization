/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.h
 *  @author Surya Singh
 *  @brief  Header file for ROS subscriber class
 *  @date   March 1, 2025
 **/

#ifndef ROS_COMMUNICATION_ROS_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS_SUBSCRIBER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <sensor_msgs/msg/imu.hpp>
#include <functional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "custom_sensor_msgs/msg/contact.hpp"
#include "custom_sensor_msgs/msg/contact_array.hpp"

#include "drift/estimator/inekf_estimator.h"
#include "drift/kinematics/mini_cheetah_kinematics.h"
#include "drift/utils/type_def.h"

using namespace measurement;

typedef std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> IMUQueuePair; 
typedef std::pair<VelocityQueuePtr, std::shared_ptr<std::mutex>> VelocityQueuePair; 
typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> PositionQueuePair; 
typedef std::pair<AngularVelocityQueuePtr, std::shared_ptr<std::mutex>> AngularVelocityQueuePair; 
typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> OdomQueuePair; 
typedef std::pair<LeggedKinQueuePtr, std::shared_ptr<std::mutex>> LeggedKinQueuePair; 

// Message Filters 
typedef message_filters::Subscriber<custom_sensor_msgs::msg::ContactArray> ContactMsgFilterT; 
typedef message_filters::Subscriber<sensor_msgs::msg::JointState> JointStateMsgFilterT; 
typedef std::shared_ptr<ContactMsgFilterT> ContactMsgFilterTPtr; 
typedef std::shared_ptr<JointStateMsgFilterT> JointStateMsgFilterTPtr; 
typedef message_filters::sync_policies::ApproximateTime< custom_sensor_msgs::msg::ContactArray, sensor_msgs::msg::JointState> LegKinSyncPolicy; 
typedef std::shared_ptr<message_filters::Synchronizer<LegKinSyncPolicy>> LegKinSyncPtr; 

// IMU sync 
typedef message_filters::Subscriber<sensor_msgs::msg::Imu> IMUMsgFilterT; 
typedef message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> IMUOffsetMsgFilterT; 
typedef std::shared_ptr<IMUMsgFilterT> IMUMsgFilterTPtr; 
typedef std::shared_ptr<IMUOffsetMsgFilterT> IMUOffsetMsgFilterTPtr; 
typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::msg::Imu, geometry_msgs::msg::Vector3Stamped> IMUSyncPolicy; 
typedef std::shared_ptr<message_filters::Synchronizer<IMUSyncPolicy>> IMUSyncPtr; 

// GPS Queue 
typedef std::queue<std::shared_ptr<NavSatMeasurement<double>>> GPSNavSatQueue; 
typedef std::shared_ptr<GPSNavSatQueue> GPSNavSatQueuePtr; 
typedef std::pair<GPSNavSatQueuePtr, std::shared_ptr<std::mutex>> GPSNavSatQueuePair;

namespace ros_wrapper {

class ROSSubscriber{
public:
  /// Constructor
  ROSSubscriber(rclcpp::Node::SharedPtr node);

  /// Destructor
  ~ROSSubscriber();

  /// Subscriber methods
  IMUQueuePair AddIMUSubscriber(const std::string topic_name);
  // IMUQueuePair AddFetchIMUSubscriber(const std::string imu_topic_name, const std::string offset_topic_name);
  // VelocityQueuePair AddVelocitySubscriber(const std::string topic_name);
  // VelocityQueuePair AddVelocityWithCovarianceSubscriber(const std::string topic_name);
  // VelocityQueuePair AddDifferentialDriveVelocitySubscriber(const std::string topic_name, double wheel_radius);

  // std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
  //          AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
  // AddDifferentialDriveVelocitySubscriber(const std::string topic_name, double wheel_radius, double track_width);

  VelocityQueuePair AddOdom2VelocityCallback(const std::string topic_name, const std::vector<double>& translation_odomsrc2body, const std::vector<double>& rotation_odomsrc2body);
  PositionQueuePair AddGPS2PositionSubscriber(const std::string& topic_name, const std::vector<double>& translation_gpssrc2body, const std::vector<double>& rotation_gpssrc2body, const Eigen::Vector3d& reference_position);
  PositionQueuePair AddOdom2PositionSubscriber(const std::string topic_name, const std::vector<double>& translation_odomsrc2body, const std::vector<double>& rotation_odomsrc2body);
  // VelocityQueuePair AddDifferentialDriveLinearVelocitySubscriber_Fetch(const std::string topic_name, double wheel_radius);

  // std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
  //            AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
  // AddDifferentialDriveVelocitySubscriber_Fetch(const std::string topic_name, double wheel_radius, double track_width);
  // LeggedKinQueuePair AddMiniCheetahKinematicsSubscriber(const std::string contact_topic_name, const std::string encoder_topic_name);

  void StartSubscribingThread();

private:
  /// Callback functions
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg, const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue);
  // void VelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg, const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);
  // void VelocityWithCovarianceCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr vel_msg, const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue);
  // void DifferentialEncoder2VelocityCallback(const sensor_msgs::msg::JointState::SharedPtr encoder_msg, const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue, double wheel_radius);
  // void DifferentialEncoder2VelocityCallback(const sensor_msgs::msg::JointState::SharedPtr encoder_msg, const std::shared_ptr<std::mutex>& vel_mutex, const std::shared_ptr<std::mutex>& ang_vel_mutex, VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue, double wheel_radius, double track_width);
  // void DifferentialEncoder2LinearVelocityCallback_Fetch(const sensor_msgs::msg::JointState::SharedPtr encoder_msg, const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue, double wheel_radius);
  // void DifferentialEncoder2VelocityCallback_Fetch(const sensor_msgs::msg::JointState::SharedPtr encoder_msg, const std::shared_ptr<std::mutex>& vel_mutex, const std::shared_ptr<std::mutex>& ang_vel_mutex, VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue, double wheel_radius, double track_width);
  // void MiniCheetahKinCallBack(const custom_sensor_msgs::msg::ContactArray::ConstSharedPtr contact_msg, const sensor_msgs::msg::JointState::ConstSharedPtr encoder_msg, const std::shared_ptr<std::mutex>& mutex, LeggedKinQueuePtr& kin_queue);
  // // void FetchIMUCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_msg, const geometry_msgs::msg::Vector3Stamped::SharedPtr imu_offset_msg, const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue);
  // void FetchIMUCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg, const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr imu_offset_msg, const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue);
  void Odom2VelocityCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg, const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue, int odom_src_id);
  void Odom2PositionCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg, const std::shared_ptr<std::mutex>& position_mutex, OdomQueuePtr& position_queue);
  void GPS2PositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg, const std::shared_ptr<std::mutex>& position_mutex, OdomQueuePtr& position_queue, const Eigen::Vector3d& reference_position);

  void RosSpin();

  rclcpp::Node::SharedPtr node_;  // The ROS 2 node handle
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list_;  // List of subscribers

  std::vector<ContactMsgFilterTPtr> contact_subscriber_list_;
  std::vector<JointStateMsgFilterTPtr> joint_state_subscriber_list_;

  std::vector<IMUMsgFilterTPtr> imu_subscriber_list_;
  std::vector<IMUOffsetMsgFilterTPtr> imu_offset_subscriber_list_;

  // Measurement queue list
  std::vector<IMUQueuePtr> imu_queue_list_;  // List of IMU queue pointers
  std::vector<VelocityQueuePtr> vel_queue_list_;  // List of velocity queue pointers
  std::vector<OdomQueuePtr> position_queue_list_;  // List of pose queue pointers
  std::vector<AngularVelocityQueuePtr> ang_vel_queue_list_;  // List of angular velocity queue pointers
  std::vector<LeggedKinQueuePtr> kin_queue_list_;  // List of kinematics queue pointers
  std::vector<LegKinSyncPtr> leg_kin_sync_list_;
  std::vector<IMUSyncPtr> imu_sync_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;  // List of mutexes
  std::unordered_map<int, OdomMeasurementPtr> prev_odom_map_;  // odom_src_id -> prev_odom_measurement

  Eigen::Matrix4d odom_src_to_body_;  // Camera to body transformation matrix
  Eigen::Matrix4d gps_src_to_body_;

  bool thread_started_;  // Flag of the thread started, true for started, false for not started
  std::thread subscribing_thread_;

  // ROS 2 Multi-threaded Executor (equivalent to MultiThreadedSpinner in ROS 1)
  rclcpp::executors::MultiThreadedExecutor executor_;

  int odom_src_id_ = 0;  // Keep track of the odom source ID, start from 0 and increment by 1 for each new odom source
};

} // namespace ros_wrapper

#endif
