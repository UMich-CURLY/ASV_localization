/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   wamv_gps_ros2.cpp
 *  @author Surya Singh
 *  @brief  Test file for WAMV robot setup (IMU Propagation +
 *  GPS based Position Correction)
 *  @date   March 1, 2025
 **/

 #include <rclcpp/rclcpp.hpp>
 #include <filesystem>
 #include <iostream>
 #include <yaml-cpp/yaml.h>
 #include <thread>
 #include <mutex>
 
 #include "communication/ros2_publisher.h"
 #include "communication/ros2_subscriber.h"
 #include "drift/estimator/inekf_estimator.h"
 
 using namespace std;
 using namespace state;
 using namespace estimator;
 
 int main(int argc, char** argv) {
     // Initialize ROS2
     rclcpp::init(argc, argv);
     auto node = std::make_shared<rclcpp::Node>("wamv_gps_ros2");
 
     std::cout << "The subscriber is on!" << std::endl;
 
     // Create ONE executor and add the node
     rclcpp::executors::MultiThreadedExecutor executor;
     executor.add_node(node);
 
     // Load configuration
     std::string file{__FILE__};
     std::string project_dir{file.substr(0, file.rfind("ROS2/drift_ros2/examples/"))};
     std::cout << "Project directory: " << project_dir << std::endl;
 
     std::string ros_config_file = project_dir + "/ROS2/drift_ros2/config/wamv_gps_ros2/ros_comm.yaml";
     YAML::Node config = YAML::LoadFile(ros_config_file);
     
     std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
     std::string gps_topic = config["subscribers"]["gps_topic"].as<std::string>();
 
     auto translation_gpssrc2body = config["subscribers"]["translation_gps_source_to_body"].as<std::vector<double>>();
 
    //  Eigen::Vector3d reference_position;
    //  try {
    //      const auto& ref_pos_node = config["subscribers"]["reference_position"];
    //      reference_position = Eigen::Vector3d(ref_pos_node[0].as<double>(), ref_pos_node[1].as<double>(), ref_pos_node[2].as<double>());
    //  } catch (const YAML::Exception& e) {
    //      throw std::runtime_error("Error reading reference_position: expected a sequence of 3 doubles.");
    //  }
 
     // Create ROS2 subscribers
     auto ros_sub = std::make_shared<ros_wrapper::ROSSubscriber>(node);
     auto qimu_and_mutex = ros_sub->AddIMUSubscriber(imu_topic);
     auto qimu = qimu_and_mutex.first;
     auto qimu_mutex = qimu_and_mutex.second;

    
 
     // Wait until the first IMU message arrives
     std::vector<double> rotation_gpssrc2body;
     {
         RCLCPP_INFO(node->get_logger(), "Waiting for first IMU message to initialize rotation...");
         rclcpp::Rate rate(10); // 10 Hz checking
         while (rclcpp::ok()) {
             {
                 std::lock_guard<std::mutex> lock(*qimu_mutex);
                 if (!qimu->empty()) {
                     auto imu_msg = qimu->front();
                     auto q = imu_msg->get_quaternion();
                     rotation_gpssrc2body = {q.w(), q.x(), q.y(), q.z()};
                     RCLCPP_INFO(node->get_logger(), "Got initial IMU orientation for rotation_gpssrc2body.");
                     break;
                 }
             }
             executor.spin_some();  // Use spin_some here to process callbacks without blocking
             rate.sleep();
         }
     }
    //  auto rotation_gpssrc2body = config["subscribers"]["rotation_gps_source_to_body"].as<std::vector<double>>();
 
     auto qp_and_mutex = ros_sub->AddGPS2PositionSubscriber(
         gps_topic, translation_gpssrc2body, rotation_gpssrc2body);
     auto qp = qp_and_mutex.first;
     auto qp_mutex = qp_and_mutex.second;
 
     // Create state estimator
     inekf::ErrorType error_type = RightInvariant;
     InekfEstimator inekf_estimator(error_type, project_dir + "/config/wamv_gps_ros2/inekf_estimator.yaml");
 
     inekf_estimator.add_imu_propagation(qimu, qimu_mutex, project_dir + "/config/wamv_gps_ros2/imu_propagation.yaml");
     inekf_estimator.add_position_correction(qp, qp_mutex, project_dir + "/config/wamv_gps_ros2/position_correction.yaml");
    //  inekf_estimator.add_orientation_correction(q_newimu, q_newimu_mutex, project_dir + "/config/wamv_gps_ros2/orientation_correction.yaml");
 
     auto robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
     auto robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();
 
     // Create ROS2 publisher
     auto ros_pub = std::make_shared<ros_wrapper::ROSPublisher>(
         node, robot_state_queue_ptr, robot_state_queue_mutex_ptr, ros_config_file);
 
     // Start the publisher thread
     ros_pub->StartPublishingThread();
 
     // Start estimator thread
     std::thread estimator_thread([&]() {
         rclcpp::Rate rate(5000); // 5000 Hz
         while (rclcpp::ok()) {
             if (inekf_estimator.is_enabled()) {
                 inekf_estimator.RunOnce();
             } else {
                 if (inekf_estimator.BiasInitialized()) {
                     inekf_estimator.InitState();
                 } else {
                     inekf_estimator.InitBias();
                 }
             }
             rate.sleep();
         }
     });
 
     // Spin the executor (this call blocks until shutdown)
     executor.spin();
 
     // Cleanup after shutdown
     if (estimator_thread.joinable()) {
         estimator_thread.join();
     }
 
     rclcpp::shutdown();
     return 0;
 }
 
