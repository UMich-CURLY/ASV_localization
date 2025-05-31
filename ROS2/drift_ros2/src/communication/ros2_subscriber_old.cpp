/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ros2_subsriber.cpp
 *  @author Surya Singh
 *  @brief  Source file for ROS subscriber class
 *  @date   March 1, 2025
 **/


#include "communication/ros2_subscriber.h"

namespace ros_wrapper {

ROSSubscriber::ROSSubscriber(rclcpp::Node::SharedPtr node)
    : node_(node), thread_started_(false) {}

ROSSubscriber::~ROSSubscriber() {
    if (thread_started_) {
        subscribing_thread_.join();
    }
    subscriber_list_.clear();
    contact_subscriber_list_.clear();
    joint_state_subscriber_list_.clear();
    imu_queue_list_.clear();
}

IMUQueuePair ROSSubscriber::AddIMUSubscriber(const std::string topic_name) {
    std::cout << "Subscribing to IMU topic: " << topic_name << std::endl;
  
    IMUQueuePtr imu_queue_ptr = std::make_shared<IMUQueue>();
    mutex_list_.emplace_back(std::make_shared<std::mutex>());
  
    subscriber_list_.push_back(
        node_->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, 1000,
            std::bind(&ROSSubscriber::IMUCallback, this, std::placeholders::_1,
                      mutex_list_.back(), imu_queue_ptr)));
  
    imu_queue_list_.push_back(imu_queue_ptr);
    return {imu_queue_ptr, mutex_list_.back()};
  }

// IMUQueuePair ROSSubscriber::AddFetchIMUSubscriber(
//     const std::string imu_topic_name, const std::string offset_topic_name) {
//   std::cout << "Subscribing to IMU topic: " << imu_topic_name << std::endl;
//   // Create a new queue for data buffers
//   IMUQueuePtr imu_queue_ptr = std::make_shared<IMUQueue>();

//   // Initialize a new mutex for this subscriber
//   mutex_list_.emplace_back(std::make_shared<std::mutex>());

//   imu_subscriber_list_.push_back(
//       std::make_shared<IMUMsgFilterT>(node_, imu_topic_name, 1));
//   imu_offset_subscriber_list_.push_back(
//       std::make_shared<IMUOffsetMsgFilterT>(node_, offset_topic_name, 1));

//   imu_sync_list_.push_back(
//       std::make_shared<message_filters::Synchronizer<IMUSyncPolicy>>(
//           IMUSyncPolicy(10), *imu_subscriber_list_.back(),
//           *imu_offset_subscriber_list_.back()));

  
//   imu_sync_list_.back()->registerCallback(
//     std::bind(&ROSSubscriber::FetchIMUCallBack, this, 
//               std::placeholders::_1, std::placeholders::_2, 
//               mutex_list_.back(), std::ref(imu_queue_ptr)));



//   imu_queue_list_.push_back(imu_queue_ptr);

//   return {imu_queue_ptr, mutex_list_.back()};
// }

// LeggedKinQueuePair ROSSubscriber::AddMiniCheetahKinematicsSubscriber(
//     const std::string contact_topic_name,
//     const std::string encoder_topic_name) {
//   std::cout << "Subscribing to contact topic: " << contact_topic_name << std::endl;
//   std::cout << "Subscribing to encoder topic: " << encoder_topic_name << std::endl;
 
//   LeggedKinQueuePtr kin_queue_ptr = std::make_shared<LeggedKinQueue>();

//   mutex_list_.emplace_back(std::make_shared<std::mutex>());

//   contact_subscriber_list_.push_back(
//       std::make_shared<ContactMsgFilterT>(node_, contact_topic_name, 1));
//   joint_state_subscriber_list_.push_back(
//       std::make_shared<JointStateMsgFilterT>(node_, encoder_topic_name, 1));

//   leg_kin_sync_list_.push_back(
//       std::make_shared<message_filters::Synchronizer<LegKinSyncPolicy>>(
//           LegKinSyncPolicy(10), *contact_subscriber_list_.back(),
//           *joint_state_subscriber_list_.back()));

//   leg_kin_sync_list_.back()->registerCallback(
//       std::bind(&ROSSubscriber::MiniCheetahKinCallBack, this, std::placeholders::_1, std::placeholders::_2,
//                 mutex_list_.back(), std::ref(kin_queue_ptr)));

//   kin_queue_list_.push_back(kin_queue_ptr);

//   return {kin_queue_ptr, mutex_list_.back()};
// }

// VelocityQueuePair ROSSubscriber::AddVelocitySubscriber(const std::string topic_name) {
//     VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();
//     mutex_list_.emplace_back(std::make_shared<std::mutex>());
  
//     subscriber_list_.push_back(
//         node_->create_subscription<geometry_msgs::msg::TwistStamped>(
//             topic_name, 1000,
//             std::bind(&ROSSubscriber::VelocityCallback, this, std::placeholders::_1,
//                       mutex_list_.back(), vel_queue_ptr)));
  
//     vel_queue_list_.push_back(vel_queue_ptr);
//     return {vel_queue_ptr, mutex_list_.back()};
//   }
  
//   VelocityQueuePair ROSSubscriber::AddVelocityWithCovarianceSubscriber(const std::string topic_name) {
//     VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();
//     mutex_list_.emplace_back(std::make_shared<std::mutex>());
  
//     subscriber_list_.push_back(
//         node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
//             topic_name, 1000,
//             std::bind(&ROSSubscriber::VelocityWithCovarianceCallback, this, std::placeholders::_1,
//                       mutex_list_.back(), vel_queue_ptr)));
  
//     vel_queue_list_.push_back(vel_queue_ptr);
//     return {vel_queue_ptr, mutex_list_.back()};
//   }

// VelocityQueuePair ROSSubscriber::AddDifferentialDriveVelocitySubscriber(
//     const std::string topic_name, double wheel_radius) {
//   std::cout << "Subscribing to wheel encoder topic: " << topic_name << std::endl;
 
//   VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();

//   mutex_list_.emplace_back(std::make_shared<std::mutex>());

//   // subscriber_list_.push_back(
//   //     node_->create_subscription<sensor_msgs::msg::JointState>(
//   //         topic_name, 1000,
//   //         std::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback, this,
//   //                   std::placeholders::_1, mutex_list_.back(), vel_queue_ptr, wheel_radius)));
//   subscriber_list_.push_back(
//     node_->create_subscription<sensor_msgs::msg::JointState>(
//         topic_name, 1000,
//         [this, &vel_queue_ptr, wheel_radius](const sensor_msgs::msg::JointState::SharedPtr encoder_msg) {
//             // Capturing 'this' to access the member variable mutex_list_
//             DifferentialEncoder2VelocityCallback(encoder_msg, this->mutex_list_.back(), vel_queue_ptr, wheel_radius);
//         }
//     )
//   );


//   vel_queue_list_.push_back(vel_queue_ptr);

//   return {vel_queue_ptr, mutex_list_.back()};
// }

// std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
//            AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
// ROSSubscriber::AddDifferentialDriveVelocitySubscriber(
//     const std::string topic_name, double wheel_radius, double track_width) {
//   std::cout << "Subscribing to wheel encoder topic: " << topic_name << std::endl;

//   VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();
//   AngularVelocityQueuePtr ang_vel_queue_ptr = std::make_shared<AngularVelocityQueue>();

//   mutex_list_.emplace_back(std::make_shared<std::mutex>());
//   auto vel_mutex = mutex_list_.back();
//   mutex_list_.emplace_back(std::make_shared<std::mutex>());
//   auto ang_vel_mutex = mutex_list_.back();

//   // subscriber_list_.push_back(
//   //     node_->create_subscription<sensor_msgs::msg::JointState>(
//   //         topic_name, 1000,
//   //         std::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback, this,
//   //                   std::placeholders::_1, vel_mutex, ang_vel_mutex, vel_queue_ptr,
//   //                   ang_vel_queue_ptr, wheel_radius, track_width)));
//   subscriber_list_.push_back(
//     node_->create_subscription<sensor_msgs::msg::JointState>(
//         topic_name, 1000,
//         [this, &vel_mutex, &ang_vel_mutex, &vel_queue_ptr, &ang_vel_queue_ptr, wheel_radius, track_width](const sensor_msgs::msg::JointState::SharedPtr encoder_msg) {
//             // Explicitly calling the correct callback
//             DifferentialEncoder2VelocityCallback(encoder_msg, vel_mutex, ang_vel_mutex, vel_queue_ptr, ang_vel_queue_ptr, wheel_radius, track_width);
//         }
//     )
//   );


//   vel_queue_list_.push_back(vel_queue_ptr);
//   ang_vel_queue_list_.push_back(ang_vel_queue_ptr);

//   return {vel_queue_ptr, vel_mutex, ang_vel_queue_ptr, ang_vel_mutex};
// }

// VelocityQueuePair
// ROSSubscriber::AddDifferentialDriveLinearVelocitySubscriber_Fetch(
//     const std::string topic_name, double wheel_radius) {
//   std::cout << "Subscribing to wheel encoder topic: " << topic_name << std::endl;

//   VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();

//   mutex_list_.emplace_back(std::make_shared<std::mutex>());

//   subscriber_list_.push_back(
//       node_->create_subscription<sensor_msgs::msg::JointState>(
//           topic_name, 1000,
//           std::bind(&ROSSubscriber::DifferentialEncoder2LinearVelocityCallback_Fetch,
//                     this, std::placeholders::_1, mutex_list_.back(), vel_queue_ptr, wheel_radius)));

//   return {vel_queue_ptr, mutex_list_.back()};
// }

VelocityQueuePair ROSSubscriber::AddOdom2VelocityCallback(const std::string topic_name,
    const std::vector<double>& translation_odomsrc2body,
    const std::vector<double>& rotation_odomsrc2body) {
  std::cout << "Subscribing to odometry topic: " << topic_name << std::endl;
  VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();
  mutex_list_.emplace_back(std::make_shared<std::mutex>());

  odom_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(rotation_odomsrc2body[0], rotation_odomsrc2body[1],
                                             rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
  odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(
      translation_odomsrc2body[0], translation_odomsrc2body[1], translation_odomsrc2body[2]);

  prev_odom_map_[odom_src_id_] = nullptr;

  subscriber_list_.push_back(
      node_->create_subscription<nav_msgs::msg::Odometry>(
          topic_name, 1000,
          std::bind(&ROSSubscriber::Odom2VelocityCallback, this, std::placeholders::_1,
                    mutex_list_.back(), vel_queue_ptr, odom_src_id_)));
 
  odom_src_id_ += 1;
  vel_queue_list_.push_back(vel_queue_ptr);
  return {vel_queue_ptr, mutex_list_.back()};
}

PositionQueuePair ROSSubscriber::AddOdom2PositionSubscriber(const std::string topic_name,
    const std::vector<double>& translation_odomsrc2body,
    const std::vector<double>& rotation_odomsrc2body) {
  std::cout << "Subscribing to odometry topic: " << topic_name << std::endl;
  OdomQueuePtr position_queue_ptr = std::make_shared<OdomQueue>();
  mutex_list_.emplace_back(std::make_shared<std::mutex>());

  odom_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(rotation_odomsrc2body[0], rotation_odomsrc2body[1],
                                             rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
  odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(
      translation_odomsrc2body[0], translation_odomsrc2body[1], translation_odomsrc2body[2]);

  subscriber_list_.push_back(
      node_->create_subscription<nav_msgs::msg::Odometry>(
          topic_name, 1000,
          std::bind(&ROSSubscriber::Odom2PositionCallback, this, std::placeholders::_1,
                    mutex_list_.back(), position_queue_ptr)));

  position_queue_list_.push_back(position_queue_ptr);
  return {position_queue_ptr, mutex_list_.back()};
}

PositionQueuePair ROSSubscriber::AddGPS2PositionSubscriber(
    const std::string& topic_name,
    const std::vector<double>& translation_gpssrc2body,
    const std::vector<double>& rotation_gpssrc2body,
    const Eigen::Vector3d& reference_position) {  // Reference lat, lon, alt
  std::cout << "Subscribing to GPS topic: " << topic_name << std::endl;

  // Create a new queue for data buffers
  OdomQueuePtr position_queue_ptr = std::make_shared<OdomQueue>();

  // Initialize a new mutex for this subscriber
  auto mutex_ptr = std::make_shared<std::mutex>();
  mutex_list_.push_back(mutex_ptr);

  // Calculate the transformation from GPS source to body
  gps_src_to_body_ = Eigen::Matrix4d::Identity();
  Eigen::Quaternion<double> orientation_quat(
      rotation_gpssrc2body[0], rotation_gpssrc2body[1],
      rotation_gpssrc2body[2], rotation_gpssrc2body[3]);
  gps_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
  gps_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(
      {translation_gpssrc2body[0], translation_gpssrc2body[1], translation_gpssrc2body[2]});

  // Create the subscriber for GPS data
  auto sub = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_name, 10,
      std::bind(&ROSSubscriber::GPS2PositionCallback, this, std::placeholders::_1,
                mutex_ptr, position_queue_ptr, reference_position));

  subscriber_list_.push_back(sub);

  // Keep the ownership of the data queue in this class
  position_queue_list_.push_back(position_queue_ptr);

  return {position_queue_ptr, mutex_ptr};
}

// std::tuple<VelocityQueuePtr, std::shared_ptr<std::mutex>,
//            AngularVelocityQueuePtr, std::shared_ptr<std::mutex>>
// ROSSubscriber::AddDifferentialDriveVelocitySubscriber_Fetch(
//     const std::string topic_name, double wheel_radius, double track_width) {
//   std::cout << "Subscribing to wheel encoder topic: " << topic_name << std::endl;

//   // Create new queues for velocity data
//   VelocityQueuePtr vel_queue_ptr = std::make_shared<VelocityQueue>();
//   AngularVelocityQueuePtr ang_vel_queue_ptr = std::make_shared<AngularVelocityQueue>();

//   // Initialize new mutexes for these subscribers
//   auto vel_mutex = std::make_shared<std::mutex>();
//   auto ang_vel_mutex = std::make_shared<std::mutex>();
//   mutex_list_.push_back(vel_mutex);
//   mutex_list_.push_back(ang_vel_mutex);

//   // Create the subscriber
//   auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
//       topic_name, 10,
//       std::bind(&ROSSubscriber::DifferentialEncoder2VelocityCallback_Fetch, this,
//                 std::placeholders::_1, vel_mutex, ang_vel_mutex, vel_queue_ptr,
//                 ang_vel_queue_ptr, wheel_radius, track_width));

//   subscriber_list_.push_back(sub);

//   // Keep the ownership of the data queue in this class
//   vel_queue_list_.push_back(vel_queue_ptr);
//   ang_vel_queue_list_.push_back(ang_vel_queue_ptr);

//   return {vel_queue_ptr, vel_mutex, ang_vel_queue_ptr, ang_vel_mutex};
// };

void ROSSubscriber::StartSubscribingThread() {
    subscribing_thread_ = std::thread([this] { this->RosSpin(); });
    thread_started_ = true;
}

void ROSSubscriber::IMUCallback(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg,
    const std::shared_ptr<std::mutex>& mutex, IMUQueuePtr& imu_queue) {
  std::shared_ptr<ImuMeasurement<double>> imu_measurement = std::make_shared<ImuMeasurement<double>>();

  // imu_measurement->set_header(
  //     imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1000000000.0,
  //     imu_msg->header.frame_id);
  imu_measurement->set_header(
    0,  // Sequence number (can be set to a default value like 0)
    imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1000000000.0,  // Timestamp as double
    imu_msg->header.frame_id  // Frame ID as string
  );
  imu_measurement->set_angular_velocity(imu_msg->angular_velocity.x,
                                        imu_msg->angular_velocity.y,
                                        imu_msg->angular_velocity.z);
  imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z);
  if (Eigen::Vector4d({imu_msg->orientation.w, imu_msg->orientation.x,
                       imu_msg->orientation.y, imu_msg->orientation.z}).norm() != 0) {
    imu_measurement->set_quaternion(
        imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y,
        imu_msg->orientation.z);
  }

  std::lock_guard<std::mutex> lock(*mutex);
  imu_queue->push(imu_measurement);
}

void ROSSubscriber::VelocityCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg,
    const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue) {
  std::shared_ptr<VelocityMeasurement<double>> vel_measurement = std::make_shared<VelocityMeasurement<double>>();

  vel_measurement->set_header(
      0,  // Sequence number (can be set to a default value like 0)
      vel_msg->header.stamp.sec + vel_msg->header.stamp.nanosec / 1000000000.0,
      vel_msg->header.frame_id);

  vel_measurement->set_velocity(vel_msg->twist.linear.x,
                                vel_msg->twist.linear.y,
                                vel_msg->twist.linear.z);

  std::lock_guard<std::mutex> lock(*mutex);
  vel_queue->push(vel_measurement);
}

// void ROSSubscriber::VelocityWithCovarianceCallback(
//     const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr vel_msg,
//     const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue) {
//   // Create a velocity measurement object
//   std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
//       new VelocityMeasurement<double>);

//   // Set headers and time stamps
//   vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       vel_msg->header.stamp.sec + vel_msg->header.stamp.nanosec / 1000000000.0,
//       vel_msg->header.frame_id);

//   vel_measurement->set_velocity(vel_msg->twist.twist.linear.x,
//                                 vel_msg->twist.twist.linear.y,
//                                 vel_msg->twist.twist.linear.z);
//   std::lock_guard<std::mutex> lock(*mutex);
//   vel_queue->push(vel_measurement);
// }

// void ROSSubscriber::DifferentialEncoder2VelocityCallback(
//     const sensor_msgs::msg::JointState::SharedPtr encoder_msg,
//     const std::shared_ptr<std::mutex>& mutex, VelocityQueuePtr& vel_queue,
//     double wheel_radius) {
//   // Create a velocity measurement object
//   std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
//       new VelocityMeasurement<double>);

//   // Set headers and time stamps
//   vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1000000000.0,
//       encoder_msg->header.frame_id);

//   double vr = (encoder_msg->velocity[1] + encoder_msg->velocity[3]) / 2.0
//               * wheel_radius;
//   double vl = (encoder_msg->velocity[0] + encoder_msg->velocity[2]) / 2.0
//               * wheel_radius;
//   double vx = (vr + vl) / 2.0;

//   vel_measurement->set_velocity(vx, 0, 0);
//   std::lock_guard<std::mutex> lock(*mutex);
//   vel_queue->push(vel_measurement);
// }

// void ROSSubscriber::DifferentialEncoder2VelocityCallback(
//     const sensor_msgs::msg::JointState::SharedPtr encoder_msg,
//     const std::shared_ptr<std::mutex>& vel_mutex,
//     const std::shared_ptr<std::mutex>& ang_vel_mutex,
//     VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
//     double wheel_radius, double track_width) {
//   // Create velocity and angular velocity measurement objects
//   std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
//       new VelocityMeasurement<double>);
//   std::shared_ptr<AngularVelocityMeasurement<double>> ang_vel_measurement(
//       new AngularVelocityMeasurement<double>);

//   // Set headers and time stamps
//   vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1000000000.0,
//       encoder_msg->header.frame_id);

//   ang_vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1000000000.0,
//       encoder_msg->header.frame_id);

//   double vr = (encoder_msg->velocity[1] + encoder_msg->velocity[3]) / 2.0
//               * wheel_radius;
//   double vl = (encoder_msg->velocity[0] + encoder_msg->velocity[2]) / 2.0
//               * wheel_radius;
//   double vx = (vr + vl) / 2.0;
//   double omega_z = (vr - vl) / track_width;

//   vel_measurement->set_velocity(vx, 0, 0);
//   ang_vel_measurement->set_angular_velocity(0, 0, omega_z);

//   {
//     std::lock_guard<std::mutex> lock(*vel_mutex);
//     vel_queue->push(vel_measurement);
//   }
//   {
//     std::lock_guard<std::mutex> lock(*ang_vel_mutex);
//     ang_vel_queue->push(ang_vel_measurement);
//   }
// }

// void ROSSubscriber::DifferentialEncoder2LinearVelocityCallback_Fetch(
//     const sensor_msgs::msg::JointState::SharedPtr encoder_msg,
//     const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
//     double wheel_radius) {
//   // Create a velocity measurement object
//   std::shared_ptr<VelocityMeasurement<double>> vel_measurement(
//       new VelocityMeasurement<double>);

//   // Set headers and time stamps
//   vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1000000000.0,
//       encoder_msg->header.frame_id);

//   // Fetch
//   if (encoder_msg->velocity.size() <= 2) {
//     return;
//   }

//   double vr = encoder_msg->velocity[1] * wheel_radius;
//   double vl = encoder_msg->velocity[0] * wheel_radius;
//   double vx = (vr + vl) / 2.0;

//   vel_measurement->set_velocity(vx, 0, 0);
//   std::lock_guard<std::mutex> lock(*vel_mutex);
//   vel_queue->push(vel_measurement);
// }

// void ROSSubscriber::DifferentialEncoder2VelocityCallback_Fetch(
//     const sensor_msgs::msg::JointState::SharedPtr encoder_msg,
//     const std::shared_ptr<std::mutex>& vel_mutex,
//     const std::shared_ptr<std::mutex>& ang_vel_mutex,
//     VelocityQueuePtr& vel_queue, AngularVelocityQueuePtr& ang_vel_queue,
//     double wheel_radius, double track_width) {
 
//   auto vel_measurement = std::make_shared<VelocityMeasurement<double>>();
//   auto ang_vel_measurement = std::make_shared<AngularVelocityMeasurement<double>>();

//   vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1e9,
//       encoder_msg->header.frame_id);

//   ang_vel_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       encoder_msg->header.stamp.sec + encoder_msg->header.stamp.nanosec / 1e9,
//       encoder_msg->header.frame_id);

//   if (encoder_msg->velocity.size() <= 2) {
//     return;
//   }

//   double vr = encoder_msg->velocity[1] * wheel_radius;
//   double vl = encoder_msg->velocity[0] * wheel_radius;
//   double vx = (vr + vl) / 2.0;
//   double omega_z = (vr - vl) / track_width;

//   vel_measurement->set_velocity(vx, 0, 0);
//   ang_vel_measurement->set_angular_velocity(0, 0, omega_z);

//   std::scoped_lock lock(*vel_mutex, *ang_vel_mutex);
//   vel_queue->push(vel_measurement);
//   ang_vel_queue->push(ang_vel_measurement);
// }

// void ROSSubscriber::MiniCheetahKinCallBack(
//     const custom_sensor_msgs::msg::ContactArray::ConstSharedPtr contact_msg,
//     const sensor_msgs::msg::JointState::ConstSharedPtr encoder_msg,
//     const std::shared_ptr<std::mutex>& mutex, LeggedKinQueuePtr& kin_queue) {

//   auto kin_measurement = std::make_shared<kinematics::MiniCheetahKinematics>();
//   kin_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       contact_msg->header.stamp.sec + contact_msg->header.stamp.nanosec / 1e9,
//       contact_msg->header.frame_id);

//   Eigen::Matrix<bool, 4, 1> ct_msg;
//   ct_msg << contact_msg->contacts[0].indicator,
//       contact_msg->contacts[1].indicator, contact_msg->contacts[2].indicator,
//       contact_msg->contacts[3].indicator;
//   kin_measurement->set_contact(ct_msg);

//   Eigen::Matrix<double, 12, 1> js_msg;
//   for (int i = 0; i < 12; i++) {
//     js_msg[i] = encoder_msg->position[i];
//   }
//   kin_measurement->set_joint_state(js_msg);

//   Eigen::Matrix<double, 12, 1> jsvel_msg;
//   for (int i = 0; i < 12; i++) {
//     jsvel_msg[i] = encoder_msg->velocity[i];
//   }
//   kin_measurement->set_joint_state_velocity(jsvel_msg);

//   std::lock_guard<std::mutex> lock(*mutex);
//   kin_queue->push(kin_measurement);
// }

// void ROSSubscriber::FetchIMUCallBack(
//   const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
//   const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr imu_offset_msg,
//   const std::shared_ptr<std::mutex>& mutex, 
//   IMUQueuePtr& imu_queue) {

//   auto imu_measurement = std::make_shared<ImuMeasurement<double>>();

//   imu_measurement->set_header(
//       0,  // Sequence number (can be set to a default value like 0)
//       imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9,
//       imu_msg->header.frame_id);

//   imu_measurement->set_angular_velocity(
//       imu_msg->angular_velocity.x + imu_offset_msg->vector.x,
//       imu_msg->angular_velocity.y + imu_offset_msg->vector.y,
//       imu_msg->angular_velocity.z + imu_offset_msg->vector.z);

//   imu_measurement->set_lin_acc(imu_msg->linear_acceleration.x,
//                                imu_msg->linear_acceleration.y,
//                                imu_msg->linear_acceleration.z);

//   Eigen::Vector4d quat(imu_msg->orientation.w, imu_msg->orientation.x,
//                        imu_msg->orientation.y, imu_msg->orientation.z);
//   if (quat.norm() != 0) {
//     imu_measurement->set_quaternion(
//         imu_msg->orientation.w, imu_msg->orientation.x,
//         imu_msg->orientation.y, imu_msg->orientation.z);
//   }

//   std::lock_guard<std::mutex> lock(*mutex);
//   imu_queue->push(imu_measurement);
// }

void ROSSubscriber::Odom2VelocityCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    const std::shared_ptr<std::mutex>& vel_mutex, VelocityQueuePtr& vel_queue,
    int odom_src_id) {
  Eigen::Vector3d translation(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
  Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  auto odom_ptr = std::make_shared<OdomMeasurement>(
      translation, quat, odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1000000000.0,
      odom_msg->header.frame_id);

  auto& prev_odom_ptr = prev_odom_map_[odom_src_id];
  if (!prev_odom_ptr) {
    prev_odom_ptr = odom_ptr;
    return;
  }

  // When we have at least two odometry data, we can calculate the velocity
  auto prev_transformation = prev_odom_ptr->get_transformation();
  double prev_time = prev_odom_ptr->get_time();
  auto curr_transformation = odom_ptr->get_transformation();
  double curr_time = odom_ptr->get_time();

  double time_diff = curr_time - prev_time;

  prev_odom_map_[odom_src_id] = odom_ptr;

  Eigen::Matrix4d transformation = odom_src_to_body_.inverse()
                                   * prev_transformation.inverse()
                                   * curr_transformation * odom_src_to_body_;
  Eigen::Matrix4d twist_se3 = transformation.log();

  auto vel_measurement = std::make_shared<VelocityMeasurement<double>>();
  vel_measurement->set_header(0, odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1000000000.0,
                              odom_msg->header.frame_id);

  vel_measurement->set_velocity(twist_se3(0, 3) / time_diff,
                                twist_se3(1, 3) / time_diff,
                                twist_se3(2, 3) / time_diff);

  std::lock_guard<std::mutex> lock(*vel_mutex);
  vel_queue->push(vel_measurement);
}

void ROSSubscriber::Odom2PositionCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    const std::shared_ptr<std::mutex>& position_mutex,
    OdomQueuePtr& position_queue) {
  std::shared_ptr<OdomMeasurement> position_measurement = std::make_shared<OdomMeasurement>();

  Eigen::Vector3d translation(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
  Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  // Create the current odometry transformation matrix
  Eigen::Matrix4d curr_transformation = Eigen::Matrix4d::Identity();
  curr_transformation.block<3, 1>(0, 3) = translation;  // Set translation

  Eigen::Matrix4d transformed_pose
      = odom_src_to_body_.inverse() * curr_transformation;

  // Update the position_measurement with the transformed translation and rotation
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  // Set headers and time stamps
  position_measurement->set_header(
      0,  // Sequence number (can be set to a default value like 0)
      odom_msg->header.stamp.sec * 1.0 + odom_msg->header.stamp.nanosec / 1e9,
      odom_msg->header.frame_id);

  position_measurement->set_translation(transformed_translation);

  position_measurement->set_transformation();

  position_mutex->lock();
  position_queue->push(position_measurement);
  position_mutex->unlock();
}

void ROSSubscriber::GPS2PositionCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    const std::shared_ptr<std::mutex>& position_mutex,
    OdomQueuePtr& position_queue, const Eigen::Vector3d& reference_position) {
  std::shared_ptr<OdomMeasurement> position_measurement = std::make_shared<OdomMeasurement>();

  measurement::NavSatMeasurement<double> navsat_measurement;
  navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
  Eigen::Vector3d enu_translation = navsat_measurement.get_enu(reference_position(0), reference_position(1), reference_position(2));

  Eigen::Matrix4d enu_transformation = Eigen::Matrix4d::Identity();
  enu_transformation.block<3, 1>(0, 3) = enu_translation;

  Eigen::Matrix4d transformed_pose = gps_src_to_body_.inverse() * enu_transformation;
  Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);

  position_measurement->set_header(
      0,  // Sequence number (can be set to a default value like 0)
      gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec / 1000000000.0,
      gps_msg->header.frame_id);
  position_measurement->set_translation(transformed_translation);
  position_measurement->set_transformation();

  std::lock_guard<std::mutex> lock(*position_mutex);
  position_queue->push(position_measurement);
}

void ROSSubscriber::RosSpin() {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node_);
    executor.spin();
    rclcpp::shutdown();
}  

} // namespace ros_wrapper
