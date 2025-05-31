/* ----------------------------------------------------------------------------
 * Copyright 2025, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   wamv_gtodom.cpp
 *  @author Surya Singh
 *  @brief  Test file for WAMV robot setup (IMU Propagation +
 *  Ground-truth based Position Correction)
 *  @date   March 1, 2025
 **/

#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "communication/ros2_publisher.h"
#include "communication/ros2_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;

int main(int argc, char** argv) {
    /// Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wamv_gtodom_ros2_old");

    std::cout << "The subscriber is on!" << std::endl;

    /// Create a ROS 2 subscriber
    ros_wrapper::ROSSubscriber ros_sub(node);
    // auto ros_sub = std::make_shared<ros_wrapper::ROSSubscriber>();

    /// Load YAML configuration file
    std::string file{__FILE__};
    std::string project_dir{file.substr(0, file.rfind("ROS2/drift_ros2/examples/"))};
    std::cout << "Project directory: " << project_dir << std::endl;

    std::string ros_config_file = project_dir + "/ROS2/drift_ros2/config/wamv_gtodom_ros2/ros_comm.yaml";
    YAML::Node config = YAML::LoadFile(ros_config_file);
    std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
    std::string odom_topic = config["subscribers"]["odom_topic"].as<std::string>();
    std::vector<double> translation_odomsrc2body = 
        config["subscribers"]["translation_odom_source_to_body"].as<std::vector<double>>();
    std::vector<double> rotation_odomsrc2body = 
        config["subscribers"]["rotation_odom_source_to_body"].as<std::vector<double>>();

    /// Add subscribers
    auto qimu_and_mutex = ros_sub.AddIMUSubscriber(imu_topic);
    auto qimu = qimu_and_mutex.first;
    auto qimu_mutex = qimu_and_mutex.second;

    auto qp_and_mutex = ros_sub.AddOdom2PositionSubscriber(
        odom_topic, translation_odomsrc2body, rotation_odomsrc2body);
    auto qp = qp_and_mutex.first;
    auto qp_mutex = qp_and_mutex.second;

    /// Start subscriber thread
    ros_sub.StartSubscribingThread();
    // ros_sub->StartSubscribingThread(ros_sub);

    /// Configure state estimator
    inekf::ErrorType error_type = RightInvariant;
    InekfEstimator inekf_estimator(
        error_type, project_dir + "/config/wamv_gtodom_ros2/inekf_estimator.yaml");

    /// Add propagation and correction methods
    inekf_estimator.add_imu_propagation(
        qimu, qimu_mutex, project_dir + "/config/wamv_gtodom_ros2/imu_propagation.yaml");
    inekf_estimator.add_position_correction(
        qp, qp_mutex, project_dir + "/config/wamv_gtodom_ros2/position_correction.yaml");

    RCLCPP_INFO(node->get_logger(), "Estimator configuration complete!");

    /// Get state queue and mutex from estimator
    RobotStateQueuePtr robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
    std::shared_ptr<std::mutex> robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();

    /// Create ROS 2 publisher
    ros_wrapper::ROSPublisher ros_pub(
        node, robot_state_queue_ptr, robot_state_queue_mutex_ptr, ros_config_file);
    
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    ros_pub.StartPublishingThread();

    /// Run state estimator
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // rclcpp::Rate loop_rate(30); // Adjust as necessary
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
        //rclcpp::spin_some(node, std::chrono::nanoseconds(0));
        executor.spin_some(); 
        //loop_rate.sleep();
    }

    // auto timer = node->create_wall_timer(
    //     std::chrono::milliseconds(33), // ~30 Hz
    //     [&inekf_estimator]() {
    //         if (inekf_estimator.is_enabled()) {
    //             inekf_estimator.RunOnce();
    //         } else {
    //             if (inekf_estimator.BiasInitialized()) {
    //                 inekf_estimator.InitState();
    //             } else {
    //                 inekf_estimator.InitBias();
    //             }
    //         }
    //     }
    // );
    
    // while (rclcpp::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    // ✅ Estimator running in its own thread
    // std::thread estimator_thread([&inekf_estimator]() {
    //     rclcpp::Rate loop_rate(30);  // ~30 Hz
    //     while (rclcpp::ok()) {
    //         if (inekf_estimator.is_enabled()) {
    //             inekf_estimator.RunOnce();
    //         } else {
    //             if (inekf_estimator.BiasInitialized()) {
    //                 inekf_estimator.InitState();
    //             } else {
    //                 inekf_estimator.InitBias();
    //             }
    //         }
    //         loop_rate.sleep();
    //     }
    // });

    // // ✅ Main thread just sleeps to prevent exiting
    // while (rclcpp::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    // // Ensure the estimator thread joins before exiting
    // estimator_thread.join();

    // while (rclcpp::ok()) {
    //     // Step behavior
    //     if (inekf_estimator.is_enabled()) {
    //         inekf_estimator.RunOnce();
    //     } else {
    //         if (inekf_estimator.BiasInitialized()) {
    //             inekf_estimator.InitState();
    //         } else {
    //             inekf_estimator.InitBias();
    //         }
    //     }
    //     rclcpp::spin_some(node);
    // }

    rclcpp::shutdown();
    return 0;
}
