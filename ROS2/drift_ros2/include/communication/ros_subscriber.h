/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.h
 *  @author Tzu-Yuan Lin
 *  @brief  Header file for ROS subscriber class
 *  @date   December 6, 2022
 **/

#ifndef ROS_COMMUNICATION_ROS_SUBSCRIBER_H
#define ROS_COMMUNICATION_ROS_SUBSCRIBER_H

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/NavSatFix.h>
#include <unsupported/Eigen/MatrixFunctions>
#include "boost/bind.hpp"
#include "custom_sensor_msgs/Contact.h"
#include "custom_sensor_msgs/ContactArray.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"

#include "drift/estimator/inekf_estimator.h"
#include "drift/kinematics/mini_cheetah_kinematics.h"
#include "drift/utils/type_def.h"

using namespace measurement;

typedef std::pair<IMUQueuePtr, std::shared_ptr<std::mutex>> IMUQueuePair; /**<
Pair of IMUQueuePtr and IMUQueue mutex. */

typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>>
    PositionQueuePair; /**< Pair of PositionQueuePtr and PositionQueue mutex. */

typedef std::pair<OdomQueuePtr, std::shared_ptr<std::mutex>> OdomQueuePair; /**<
Pair of OdomQueuePtr and OdomQueue mutex. */

// IMU sync
typedef message_filters::Subscriber<sensor_msgs::Imu>
    IMUMsgFilterT; /**< Message filter for IMU messages. */
typedef message_filters::Subscriber<geometry_msgs::Vector3Stamped>
    IMUOffsetMsgFilterT; /**< Message filter for IMU offset messages. */
typedef std::shared_ptr<IMUMsgFilterT>
    IMUMsgFilterTPtr; /**< Pointer to the IMUMsgFilterT. */
typedef std::shared_ptr<IMUOffsetMsgFilterT>
    IMUOffsetMsgFilterTPtr; /**< Pointer to the IMUOffsetMsgFilterT. */
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Imu, geometry_msgs::Vector3Stamped>
    IMUSyncPolicy; /**< Sync policy for IMU. */
typedef std::shared_ptr<message_filters::Synchronizer<IMUSyncPolicy>>
    IMUSyncPtr; /**< Pointer to the IMUSyncPolicy. */

// GPS
typedef std::queue<std::shared_ptr<NavSatMeasurement<double>>> GPSNavSatQueue;
typedef std::shared_ptr<GPSNavSatQueue> GPSNavSatQueuePtr;
typedef std::pair<GPSNavSatQueuePtr, std::shared_ptr<std::mutex>>
    GPSNavSatQueuePair;

namespace ros_wrapper {
/**
 * @class ROSSubscriber
 * @brief ROS subscriber class, which subscribes to the ROS topics and stores
 * the measurements in the queue.
 *
 */
class ROSSubscriber {
 public:
  /// @name Constructors
  /// @{
  /**
   * @brief Construct a new ROSSubscriber object
   *
   * @param[in] nh ROS node handle
   */
  ROSSubscriber(ros::NodeHandle* nh);
  /// @}

  /// @name Destructors
  /// @{
  /**
   * @brief Destroy the ROSSubscriber object
   */
  ~ROSSubscriber();
  /// @}

  /// @name ROS subscriber adders
  /// TODO: Detail what message type each subscribers subscribe to.
  /**
   * @brief Add an IMU subscriber to the given topic and return a queue pair.
   * The queue pair contains the queue and the mutex for the queue. The queue
   * stores the IMU measurements and the mutex is used to protect the queue.
   *
   *
   * @param[in] topic_name IMU topic name
   * @return IMUQueuePair IMU queue pair
   */
  IMUQueuePair AddIMUSubscriber(const std::string topic_name);


  /**
   * @brief Add GPS NavSat subscriber
   *
   * @param[in] topic_name GPS navsat topic name
   * @return GPSNavSatQueuePair navsat queue pair
   */
  PositionQueuePair AddGPS2PositionSubscriber(
      const std::string& topic_name,
      const std::vector<double>& translation_gpssrc2body,
      const std::vector<double>& rotation_gpssrc2body,
      const Eigen::Vector3d& reference_position);

  /**
   * @brief
   *
   */
  PositionQueuePair AddOdom2PositionSubscriber(
      const std::string topic_name,
      const std::vector<double>& translation_odomsrc2body,
      const std::vector<double>& rotation_odomsrc2body);


  /// @}

  /**
   * @brief Start the subscribing thread
   */
  void StartSubscribingThread();

 private:
  /// @name Callback functions
  /// @{
  /**
   * @brief IMU callback function
   *
   * @param[in] imu_msg: IMU message
   * @param[in] mutex: mutex for the buffer queue
   * @param[in] imu_queue: pointer to the buffer queue
   */
  void IMUCallback(const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
                   const std::shared_ptr<std::mutex>& mutex,
                   IMUQueuePtr& imu_queue);


  /**
   * @brief odometry to pose callback function
   *
   * @param pose_msg: odometry message
   * @param pose_mutex: mutex for the buffer queue
   * @param pose_queue: pointer to the buffer queue
   *
   */
  void Odom2PositionCallback(
      const boost::shared_ptr<const nav_msgs::Odometry>& odom_msg,
      const std::shared_ptr<std::mutex>& position_mutex,
      OdomQueuePtr& position_queue);

  void GPS2PositionCallback(
      const boost::shared_ptr<const sensor_msgs::NavSatFix>& gps_msg,
      const std::shared_ptr<std::mutex>& position_mutex,
      OdomQueuePtr& position_queue, const Eigen::Vector3d& reference_position);

  void RosSpin();

  ros::NodeHandle* nh_;                             // The ROS handle
  std::vector<ros::Subscriber> subscriber_list_;    // List of subscribers

  std::vector<IMUMsgFilterTPtr> imu_subscriber_list_;
  std::vector<IMUOffsetMsgFilterTPtr> imu_offset_subscriber_list_;

  // measurement queue list
  std::vector<IMUQueuePtr> imu_queue_list_;    // List of IMU queue pointers
  
  std::vector<OdomQueuePtr>
      position_queue_list_;    // List of pose queue pointers
  
  std::vector<IMUSyncPtr> imu_sync_list_;
  std::vector<std::shared_ptr<std::mutex>> mutex_list_;    // List of mutexes
  std::unordered_map<int, OdomMeasurementPtr>
      prev_odom_map_;                   // odom_src_id -> prev_odom_measurement
  Eigen::Matrix4d odom_src_to_body_;    // Camera to body transformation matrix
  Eigen::Matrix4d gps_src_to_body_;

  bool thread_started_;    // Flag of the thread started, true for started,
                           // false for not started
  std::thread subscribing_thread_;

  ros::MultiThreadedSpinner spinner_;

  int odom_src_id_ = 0;    // Keep track of the odom source id, start from 0 and
                           // increment by 1 for each new odom source
};

}    // namespace ros_wrapper

#endif
