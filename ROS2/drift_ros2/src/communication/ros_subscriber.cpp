/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros_subsriber.cpp
 *  @author Tzu-Yuan Lin, Tingjun Li, Justin Yu
 *  @brief  Source file for ROS subscriber class
 *  @date   May 16, 2023
 **/

#include "communication/ros_subscriber.h"

namespace ros_wrapper {


ROSSubscriber::ROSSubscriber(ros::NodeHandle* nh)
    : nh_(nh), thread_started_(false) {}


ROSSubscriber::~ROSSubscriber() {
  if (thread_started_ == true) {
    subscribing_thread_.join();
  }
  subscriber_list_.clear();
  contact_subscriber_list_.clear();
  joint_state_subscriber_list_.clear();
  imu_queue_list_.clear();
}


IMUQueuePair ROSSubscriber::AddIMUSubscriber(const std::string topic_name) {
  // Create a new queue for data buffers
  std::cout << "Subscribing to IMU topic: " << topic_name << std::endl;

  IMUQueuePtr imu_queue_ptr(new IMUQueue);
  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::Imu>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::IMUCallback, this, _1, mutex_list_.back(),
                  imu_queue_ptr)));

  // Keep the ownership of the data queue in this class
  imu_queue_list_.push_back(imu_queue_ptr);

  return {imu_queue_ptr, mutex_list_.back()};
}


PositionQueuePair ROSSubscriber::AddOdom2PositionSubscriber(
    const std::string topic_name,
    const std::vector<double>& translation_odomsrc2body,
    const std::vector<double>& rotation_odomsrc2body) {
  std::cout << "Subscribing to odometry topic: " << topic_name << std::endl;
  // Create a new queue for data buffers
  OdomQueuePtr position_queue_ptr(new OdomQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Calculate the transformation from odometry source to body
  odom_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(
      rotation_odomsrc2body[0], rotation_odomsrc2body[1],
      rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
  odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(
      {translation_odomsrc2body[0], translation_odomsrc2body[1],
       translation_odomsrc2body[2]});

  // Create the subscriber
  subscriber_list_.push_back(nh_->subscribe<nav_msgs::Odometry>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::Odom2PositionCallback, this, _1,
                  mutex_list_.back(), position_queue_ptr)));

  // Keep the ownership of the data queue in this class
  position_queue_list_.push_back(position_queue_ptr);

  return {position_queue_ptr, mutex_list_.back()};
};

PositionQueuePair ROSSubscriber::AddGPS2PositionSubscriber(
    const std::string& topic_name,
    const std::vector<double>& translation_gpssrc2body,
    const std::vector<double>& rotation_gpssrc2body,
    const Eigen::Vector3d& reference_position) {    // Reference lat, lon, alt
  std::cout << "Subscribing to GPS topic: " << topic_name << std::endl;

  // Create a new queue for data buffers
  OdomQueuePtr position_queue_ptr(new OdomQueue);

  // Initialize a new mutex for this subscriber
  mutex_list_.emplace_back(new std::mutex);

  // Calculate the transformation from odometry source to body
  gps_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(
      rotation_gpssrc2body[0], rotation_gpssrc2body[1], rotation_gpssrc2body[2],
      rotation_gpssrc2body[3]);
  gps_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  gps_src_to_body_.block<3, 1>(0, 3)
      = Eigen::Vector3d({translation_gpssrc2body[0], translation_gpssrc2body[1],
                         translation_gpssrc2body[2]});

  // Create the subscriber for GPS data
  subscriber_list_.push_back(nh_->subscribe<sensor_msgs::NavSatFix>(
      topic_name, 1000,
      boost::bind(&ROSSubscriber::GPS2PositionCallback, this, _1,
                  mutex_list_.back(), position_queue_ptr, reference_position)));

  // Keep the ownership of the data queue in this class
  position_queue_list_.push_back(position_queue_ptr);

  return {position_queue_ptr, mutex_list_.back()};
}


void ROSSubscriber::StartSubscribingThread() {
  subscribing_thread_ = std::thread([this] { this->RosSpin(); });
  thread_started_ = true;
}

void ROSSubscriber::IMUCallback(
    const boost::shared_ptr<const sensor_msgs::Imu>& imu_msg,
    const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue) {
  // Create an imu measurement object
  std::shared_ptr<ImuMeasurement<double>> imu_measurement(
      new ImuMeasurement<double>);

  // Set headers and time stamps
  imu_measurement->set_header(
      imu_msg->header.seq,
      imu_msg->header.stamp.sec + imu_msg->header.stamp.nsec / 1000000000.0,
      imu_msg->header.frame_id);
  // Set angular velocity
  imu_measurement->set_angular_velocity(imu_msg->angular_velocity.x,
                                        imu_msg->angular_velocity.y,
                                        imu_msg->angular_velocity.z);
  // Set linear acceleration
  imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
  // Set orientation estimate
  // Check if IMU has quaternion data:
  if (Eigen::Vector4d({imu_msg->orientation.w, imu_msg->orientation.x,
                       imu_msg->orientation.y, imu_msg->orientation.z})
          .norm()
      != 0) {
    imu_measurement->set_quaternion(
        imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y,
        imu_msg->orientation.z);
  }

  mutex.get()->lock();
  imu_queue->push(imu_measurement);
  mutex.get()->unlock();
}

void ROSSubscriber::Odom2PositionCallback(
    const boost::shared_ptr<const nav_msgs::Odometry>& odom_msg,
    const std::shared_ptr<std::mutex>& position_mutex,
    OdomQueuePtr& position_queue) {
  std::shared_ptr<OdomMeasurement> position_measurement(new OdomMeasurement);

  Eigen::Vector3d translation(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
  Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  // Create the current odometry transformation matrix
  Eigen::Matrix4d curr_transformation = Eigen::Matrix4d::Identity();
  curr_transformation.block<3, 1>(0, 3) = translation;    // Set translation

  Eigen::Matrix4d transformed_pose
      = odom_src_to_body_.inverse() * curr_transformation;

  // Update the position_measurement with the transformed translation and
  // rotation
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  // Set headers and time stamps
  position_measurement->set_header(
      odom_msg->header.seq,
      odom_msg->header.stamp.sec + odom_msg->header.stamp.nsec / 1000000000.0,
      odom_msg->header.frame_id);

  position_measurement->set_translation(transformed_translation);

  position_measurement->set_transformation();

  position_mutex.get()->lock();
  position_queue->push(position_measurement);
  position_mutex.get()->unlock();
}

void ROSSubscriber::GPS2PositionCallback(
    const boost::shared_ptr<const sensor_msgs::NavSatFix>& gps_msg,
    const std::shared_ptr<std::mutex>& position_mutex,
    OdomQueuePtr& position_queue, const Eigen::Vector3d& reference_position) {
  std::shared_ptr<OdomMeasurement> position_measurement(new OdomMeasurement);

  double lat0 = reference_position(0);
  double lon0 = reference_position(1);
  double alt0 = reference_position(2);

  // Convert GPS coordinates to ENU coordinates
  measurement::NavSatMeasurement<double> navsat_measurement;
  navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude,
                                   gps_msg->altitude);

  // Obtain ENU coordinates relative to the reference lat/lon/alt
  Eigen::Matrix<double, 3, 1> enu_translation
      = navsat_measurement.get_enu(lat0, lon0, alt0);

  // Set up transformation matrix for ENU translation (no rotation here as GPS
  // lacks orientation data)
  Eigen::Matrix4d enu_transformation = Eigen::Matrix4d::Identity();
  enu_transformation.block<3, 1>(0, 3) = enu_translation;

  Eigen::Matrix4d transformed_pose
      = gps_src_to_body_.inverse() * enu_transformation;

  // Update the position_measurement with the transformed translation and
  // rotation
  //   Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0,
  //   3);

  //   Eigen::Matrix4d transformed_pose = enu_transformation;
  // Update the position_measurement with the transformed translation and
  // rotation
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  // Set headers and time stamps
  position_measurement->set_header(
      gps_msg->header.seq,
      gps_msg->header.stamp.sec + gps_msg->header.stamp.nsec / 1000000000.0,
      gps_msg->header.frame_id);

  position_measurement->set_translation(transformed_translation);

  position_measurement->set_transformation();

  position_mutex.get()->lock();
  position_queue->push(position_measurement);
  position_mutex.get()->unlock();
}

void ROSSubscriber::RosSpin() {
  //   while (ros::ok()) {
  //     ros::spinOnce();
  //   }
  // spinner_.spin();

  ros::AsyncSpinner spinner(4);    // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
}

}    // namespace ros_wrapper
