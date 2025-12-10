#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <mutex>
#include <Eigen/Geometry>
#include <cmath>

#include "communication/ros2_publisher.h"
#include "communication/ros2_subscriber.h"
#include "drift/estimator/inekf_estimator.h"

using namespace std;
using namespace state;
using namespace estimator;

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wamv_gpsimu_ros2");

    // Create executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Load configuration
    std::string file{__FILE__};
    std::string project_dir{file.substr(0, file.rfind("ROS2/drift_ros2/examples/"))};
    std::cout << "Project directory: " << project_dir << std::endl;

    std::string ros_config_file = project_dir + "/ROS2/drift_ros2/config/wamv_gpsimu_ros2/ros_comm.yaml";
    YAML::Node config = YAML::LoadFile(ros_config_file);

    std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
    std::string gps_topic = config["subscribers"]["gps_topic"].as<std::string>();
    auto translation_gpssrc2body = config["subscribers"]["translation_gps_source_to_body"].as<std::vector<double>>();
    auto rotation_gpssrc2body = config["subscribers"]["rotation_gps_source_to_body"].as<std::vector<double>>();

    double heading_offset_deg = config["subscribers"]["heading_offset_deg"]
        ? config["subscribers"]["heading_offset_deg"].as<double>() : 0.0;
    constexpr double deg2rad = 3.14159265358979323846 / 180.0;
    const double heading_offset_rad = heading_offset_deg * deg2rad;
    Eigen::Quaterniond heading_offset_quat(Eigen::AngleAxisd(
        heading_offset_rad, Eigen::Vector3d::UnitZ()));
    RCLCPP_INFO(node->get_logger(), "Heading offset: %.2f deg (%.4f rad)",
        heading_offset_deg, heading_offset_rad);

    // Create ROS2 subscriber
    auto ros_sub = std::make_shared<ros_wrapper::ROSSubscriber>(node);

    // Temporary IMU subscriber to initialize rotation
    auto qimu_and_mutex = ros_sub->AddIMUSubscriber(imu_topic);
    auto qimu = qimu_and_mutex.first;
    auto qimu_mutex = qimu_and_mutex.second;

    // Add subscriber that fuses GPS + IMU to pose measurements
    auto qpose_and_mutex = ros_sub->AddGPSIMU2PoseSubscriber(
        gps_topic, imu_topic, translation_gpssrc2body, rotation_gpssrc2body, heading_offset_rad);
    auto qpose = qpose_and_mutex.first;
    auto qpose_mutex = qpose_and_mutex.second;

    // Create state estimator
    inekf::ErrorType error_type = RightInvariant;
    InekfEstimator inekf_estimator(error_type, project_dir + "/config/wamv_gpsimu_ros2/inekf_estimator.yaml");

    // Add IMU propagation and pose correction
    inekf_estimator.add_imu_propagation(qimu, qimu_mutex, project_dir + "/config/wamv_gpsimu_ros2/imu_propagation.yaml");
    inekf_estimator.add_pose_correction(qpose, qpose_mutex, project_dir + "/config/wamv_gpsimu_ros2/pose_correction.yaml");

    auto robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
    auto robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();

    // Create ROS2 publisher
    auto ros_pub = std::make_shared<ros_wrapper::ROSPublisher>(
        node, robot_state_queue_ptr, robot_state_queue_mutex_ptr, ros_config_file);

    // Start publishing thread
    ros_pub->StartPublishingThread();

    // Start estimator thread
    std::thread estimator_thread([&]() {
        rclcpp::Rate rate(5000); // 5000 Hz
        bool heading_corrected = false;
        
        while (rclcpp::ok()) {
            if (inekf_estimator.is_enabled()) {
                inekf_estimator.RunOnce();
                
                // Apply heading correction once after first state is initialized
                if (!heading_corrected && !robot_state_queue_ptr->empty()) {
                    auto state_ptr = robot_state_queue_ptr->front();
                    
                    Eigen::Quaterniond gps_quat(
                        rotation_gpssrc2body[0], rotation_gpssrc2body[1],
                        rotation_gpssrc2body[2], rotation_gpssrc2body[3]
                    );
                    Eigen::Matrix3d gps_rotation_matrix = (heading_offset_quat * gps_quat).toRotationMatrix();
                    Eigen::Matrix3d current_rotation = state_ptr->get_rotation();
                    Eigen::Matrix3d corrected_rotation = gps_rotation_matrix * current_rotation;
                    
                    state_ptr->set_rotation(corrected_rotation);
                    RCLCPP_INFO(node->get_logger(), "Applied initial heading correction");
                    heading_corrected = true;
                }
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

    executor.spin();

    // Cleanup
    if (estimator_thread.joinable()) {
        estimator_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
