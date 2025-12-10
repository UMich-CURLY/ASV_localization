#include <rclcpp/rclcpp.hpp>
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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wamv_gps_ros2");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::string file{__FILE__};
    std::string project_dir{file.substr(0, file.rfind("ROS2/drift_ros2/examples/"))};
    std::cout << "Project directory: " << project_dir << std::endl;

    std::string ros_config_file = project_dir + "/ROS2/drift_ros2/config/wamv_gps_ros2/ros_comm.yaml";
    YAML::Node config = YAML::LoadFile(ros_config_file);
    
    std::string imu_topic = config["subscribers"]["imu_topic"].as<std::string>();
    std::string gps_topic = config["subscribers"]["gps_topic"].as<std::string>();

    auto translation_gpssrc2body = config["subscribers"]["translation_gps_source_to_body"].as<std::vector<double>>();
    auto rotation_gpssrc2body = config["subscribers"]["rotation_gps_source_to_body"].as<std::vector<double>>();

    auto ros_sub = std::make_shared<ros_wrapper::ROSSubscriber>(node);
    auto qimu_and_mutex = ros_sub->AddIMUSubscriber(imu_topic);
    auto qimu = qimu_and_mutex.first;
    auto qimu_mutex = qimu_and_mutex.second;

    auto qp_and_mutex = ros_sub->AddGPS2PositionSubscriber(
        gps_topic, translation_gpssrc2body, rotation_gpssrc2body);
    auto qp = qp_and_mutex.first;
    auto qp_mutex = qp_and_mutex.second;

    inekf::ErrorType error_type = RightInvariant;
    InekfEstimator inekf_estimator(error_type, project_dir + "/config/wamv_gps_ros2/inekf_estimator.yaml");

    inekf_estimator.add_imu_propagation(qimu, qimu_mutex, project_dir + "/config/wamv_gps_ros2/imu_propagation.yaml");
    inekf_estimator.add_position_correction(qp, qp_mutex, project_dir + "/config/wamv_gps_ros2/position_correction.yaml");

    auto robot_state_queue_ptr = inekf_estimator.get_robot_state_queue_ptr();
    auto robot_state_queue_mutex_ptr = inekf_estimator.get_robot_state_queue_mutex_ptr();

    auto ros_pub = std::make_shared<ros_wrapper::ROSPublisher>(
        node, robot_state_queue_ptr, robot_state_queue_mutex_ptr, ros_config_file);

    ros_pub->StartPublishingThread();

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

    executor.spin();

    if (estimator_thread.joinable()) {
        estimator_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
