#include "communication/ros2_subscriber.h"
#include <deque>
// #include <memory>
// #include <iostream>
// #include <Eigen/Dense>

namespace {

struct TimedOrientation {
    double stamp;
    Eigen::Quaterniond orientation;
};

constexpr std::size_t kMaxOrientationBufferSize = 200;
constexpr double kMaxOrientationAgeSec = 0.05;

double StampToSeconds(const builtin_interfaces::msg::Time& stamp) {
    return stamp.sec + stamp.nanosec * 1e-9;
}

}  // namespace

namespace ros_wrapper {

// ROSSubscriber::ROSSubscriber(rclcpp::Node::SharedPtr node)
//     : rclcpp::Node("ros2_subscriber"), node_(node), thread_started_(false) {}
ROSSubscriber::ROSSubscriber(rclcpp::Node::SharedPtr node)
    : node_(node),
      thread_started_(false) {}

void ROSSubscriber::SetWorldAlignmentYaw(const double yaw_rad) {
    world_alignment_ = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    world_alignment_.normalize();
    RCLCPP_INFO(node_->get_logger(),
                "World alignment yaw set to %.6f rad (%.2f deg).",
                yaw_rad, yaw_rad * 180.0 / 3.14159265358979323846);
}

void ROSSubscriber::SetReferencePosition(const double lat_deg,
                                         const double lon_deg,
                                         const double alt_m) {
    reference_position << lat_deg, lon_deg, alt_m;
    reference_initialized = true;
    RCLCPP_INFO(
        node_->get_logger(),
        "Reference position forced from config: [%.9f, %.9f, %.3f]",
        reference_position(0),
        reference_position(1),
        reference_position(2));
}

ROSSubscriber::~ROSSubscriber() {
    if (thread_started_) {
        if (subscribing_thread_.joinable()) {
            subscribing_thread_.join();
        }
    }
    subscriber_list_.clear();
    imu_queue_list_.clear();
}

IMUQueuePair ROSSubscriber::AddIMUSubscriber(const std::string& topic_name) {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to IMU topic: %s", topic_name.c_str());

    // Create queue and mutex
    IMUQueuePtr imu_queue_ptr = std::make_shared<std::queue<std::shared_ptr<ImuMeasurement<double>>>>();
    auto mutex = std::make_shared<std::mutex>();
    mutex_list_.push_back(mutex);

    // Create ROS2 subscription
    auto callback = [this, mutex, imu_queue_ptr](const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        this->IMUCallback(imu_msg, mutex, imu_queue_ptr);
    };

    auto subscriber = node_->create_subscription<sensor_msgs::msg::Imu>(
        topic_name, 
        1000,  
        callback
    );

    subscriber_list_.push_back(subscriber);
    imu_queue_list_.push_back(imu_queue_ptr);

    return {imu_queue_ptr, mutex};
}

PositionQueuePair ROSSubscriber::AddOdom2PositionSubscriber(
    const std::string &topic_name,
    const std::vector<double> &translation_odomsrc2body,
    const std::vector<double> &rotation_odomsrc2body) {
    std::cout << "Subscribing to odometry topic: " << topic_name << std::endl;
    auto position_queue_ptr = std::make_shared<OdomQueue>();
    auto mutex = std::make_shared<std::mutex>();

    Eigen::Quaternion<double> orientation_quat(rotation_odomsrc2body[0], rotation_odomsrc2body[1],
                                               rotation_odomsrc2body[2], rotation_odomsrc2body[3]);
    odom_src_to_body_ = Eigen::Matrix4d::Identity();
    odom_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
    odom_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_odomsrc2body.data());

    auto callback = [this, mutex, position_queue_ptr](const nav_msgs::msg::Odometry::SharedPtr msg) {
        Odom2PositionCallback(msg, mutex, position_queue_ptr);
    };

    subscriber_list_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
        topic_name, 1000, callback));

    position_queue_list_.push_back(position_queue_ptr);
    return {position_queue_ptr, mutex};
}

PositionQueuePair ROSSubscriber::AddGPS2PositionSubscriber(
    const std::string &topic_name,
    const std::vector<double> &translation_gpssrc2body,
    const std::vector<double> &rotation_gpssrc2body) 
{
    std::cout << "Subscribing to GPS topic: " << topic_name << std::endl;
    
    auto position_queue_ptr = std::make_shared<OdomQueue>();
    auto mutex = std::make_shared<std::mutex>();

    // Set gps_src_to_body_ transform
    Eigen::Quaternion<double> orientation_quat(
        rotation_gpssrc2body[0], rotation_gpssrc2body[1],
        rotation_gpssrc2body[2], rotation_gpssrc2body[3]);

    gps_src_to_body_ = Eigen::Matrix4d::Identity();
    gps_src_to_body_.block<3, 3>(0, 0) = orientation_quat.toRotationMatrix();
    gps_src_to_body_.block<3, 1>(0, 3) = Eigen::Vector3d(translation_gpssrc2body.data());

    // Create a temporary subscription JUST to initialize reference_position
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr temp_sub;
    temp_sub = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_name, 10,
        [&, this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            if (!reference_initialized) {
                reference_position << msg->latitude, msg->longitude, msg->altitude;
                reference_initialized = true;
                RCLCPP_INFO(node_->get_logger(), "GPS reference position initialized: [%f, %f, %f]", 
                            reference_position(0), reference_position(1), reference_position(2));
                
                // After initializing, destroy this temp subscriber
                temp_sub.reset();
            }
        });

    // Real subscriber (pure callback assuming reference_position is ready)
    auto callback = [this, mutex, position_queue_ptr](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!reference_initialized) {
            reference_position << msg->latitude, msg->longitude, msg->altitude;
            reference_initialized = true;
            RCLCPP_INFO(node_->get_logger(), "GPS reference position initialized: [%f, %f, %f]", 
                        reference_position(0), reference_position(1), reference_position(2));
        }
    
        // RCLCPP_INFO(node_->get_logger(), "Processing GPS message: [%f, %f, %f]", msg->latitude, msg->longitude, msg->altitude);
        
        // Check if the reference position is initialized
        if (reference_initialized) {
            GPS2PositionCallback(msg, mutex, position_queue_ptr, reference_position);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Reference position still not initialized! Skipping message.");
        }
    };
    // auto callback = [this, mutex, position_queue_ptr, reference_position](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    //             GPS2PositionCallback(msg, mutex, position_queue_ptr, reference_position);
    //         };

    subscriber_list_.push_back(node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic_name, 1000, callback));

    position_queue_list_.push_back(position_queue_ptr);

    return {position_queue_ptr, mutex};
}

PoseQueuePair ROSSubscriber::AddGPSIMU2PoseSubscriber(
    const std::string &gps_topic_name,
    const std::string &imu_topic_name,
    const std::vector<double> &translation_gpssrc2body,
    const std::vector<double> &rotation_gpssrc2body,
    const std::vector<double> &rotation_imu2body)
{
    std::cout << "Subscribing to GPS: " << gps_topic_name << " and IMU: " << imu_topic_name << std::endl;

    auto pose_queue_ptr = std::make_shared<OdomQueue>();
    auto mutex = std::make_shared<std::mutex>();

    // Transform from GPS source to robot body
    Eigen::Quaterniond orientation_quat(
        rotation_gpssrc2body[0], rotation_gpssrc2body[1],
        rotation_gpssrc2body[2], rotation_gpssrc2body[3]);

    gps_src_to_body_ = Eigen::Matrix4d::Identity();
    gps_src_to_body_.block<3,3>(0,0) = orientation_quat.toRotationMatrix();
    gps_src_to_body_.block<3,1>(0,3) = Eigen::Vector3d(translation_gpssrc2body.data());

    Eigen::Quaterniond q_body_from_imu(
        rotation_imu2body[0],
        rotation_imu2body[1],
        rotation_imu2body[2],
        rotation_imu2body[3]);
    q_body_from_imu.normalize();
    const Eigen::Quaterniond q_imu_from_body = q_body_from_imu.inverse();
    const Eigen::Quaterniond world_alignment = world_alignment_;

    auto orientation_buffer =
        std::make_shared<std::deque<TimedOrientation>>();
    auto orientation_buffer_mutex = std::make_shared<std::mutex>();

    auto imu_callback =
        [orientation_buffer, orientation_buffer_mutex, q_imu_from_body,
         world_alignment](
            const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (msg->orientation_covariance[0] < 0.0) {
            return;
        }

        Eigen::Quaterniond q_world_from_imu(
            msg->orientation.w, msg->orientation.x,
            msg->orientation.y, msg->orientation.z);
        if (q_world_from_imu.squaredNorm() <= 1e-12) {
            return;
        }
        q_world_from_imu.normalize();

        Eigen::Quaterniond q_world_from_body =
            world_alignment * q_world_from_imu * q_imu_from_body;
        q_world_from_body.normalize();

        const double stamp = StampToSeconds(msg->header.stamp);
        std::lock_guard<std::mutex> lock(*orientation_buffer_mutex);
        if (!orientation_buffer->empty() &&
            stamp <= orientation_buffer->back().stamp) {
            return;
        }

        orientation_buffer->push_back({stamp, q_world_from_body});
        while (orientation_buffer->size() > kMaxOrientationBufferSize) {
            orientation_buffer->pop_front();
        }
    };

    subscriber_list_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name, 1000, imu_callback));

    auto gps_callback =
        [this, mutex, pose_queue_ptr, orientation_buffer,
         orientation_buffer_mutex](
            const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (!reference_initialized) {
            reference_position << msg->latitude, msg->longitude, msg->altitude;
            reference_initialized = true;
            RCLCPP_INFO(node_->get_logger(), "GPS reference position initialized: [%f, %f, %f]", 
                        reference_position(0), reference_position(1), reference_position(2));
        }
        const double gps_stamp = StampToSeconds(msg->header.stamp);
        Eigen::Quaterniond body_orientation;
        double orientation_age = 0.0;

        {
            std::lock_guard<std::mutex> lock(*orientation_buffer_mutex);
            while (orientation_buffer->size() >= 2 &&
                   (*orientation_buffer)[1].stamp <= gps_stamp) {
                orientation_buffer->pop_front();
            }

            if (orientation_buffer->empty() ||
                orientation_buffer->front().stamp > gps_stamp) {
                return;
            }

            orientation_age =
                gps_stamp - orientation_buffer->front().stamp;
            if (orientation_age > kMaxOrientationAgeSec) {
                return;
            }

            body_orientation = orientation_buffer->front().orientation;
        }

        GPSIMU2PoseCallback(msg, mutex, body_orientation, pose_queue_ptr,
                            reference_position);
    };
    
    subscriber_list_.push_back(node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_name, 1000, gps_callback));

    pose_queue_list_.push_back(pose_queue_ptr);
    return {pose_queue_ptr, mutex};
}

void ROSSubscriber::StartSubscribingThread() {
    subscribing_thread_ = std::thread([this]() { this->RosSpin(); });
    thread_started_ = true;
}

void ROSSubscriber::IMUCallback(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg, 
    std::shared_ptr<std::mutex> mutex, 
    IMUQueuePtr imu_queue) {

    //create an IMU measurement object
    auto imu_measurement = std::make_shared<ImuMeasurement<double>>();

    //set headers and timestamps
    imu_measurement->set_header(
        imu_msg->header.stamp.sec, 
        imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec / 1e9, 
        imu_msg->header.frame_id);

    //set angular velocity
    imu_measurement->set_angular_velocity(
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z);

    //set linear acceleration
    imu_measurement->set_lin_acc(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);

    //set quaternion if valid
    Eigen::Vector4d quat(imu_msg->orientation.w, 
                         imu_msg->orientation.x, 
                         imu_msg->orientation.y, 
                         imu_msg->orientation.z);

    if (quat.norm() != 0) {
        imu_measurement->set_quaternion(
            imu_msg->orientation.w, 
            imu_msg->orientation.x, 
            imu_msg->orientation.y, 
            imu_msg->orientation.z);
    }

    //push the measurement into the queue
    {
        std::lock_guard<std::mutex> lock(*mutex);
        imu_queue->push(imu_measurement);
    }

}


void ROSSubscriber::Odom2PositionCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg,
    std::shared_ptr<std::mutex> position_mutex, OdomQueuePtr position_queue) {
    auto position_measurement = std::make_shared<OdomMeasurement>();
    Eigen::Vector3d translation(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    Eigen::Matrix4d curr_transformation = Eigen::Matrix4d::Identity();
    curr_transformation.block<3, 1>(0, 3) = translation;
    Eigen::Matrix4d transformed_pose = odom_src_to_body_.inverse() * curr_transformation;
    Eigen::Vector3d transformed_translation = transformed_pose.block<3, 1>(0, 3);
    if (!transformed_translation.allFinite()) {
        // RCLCPP_WARN(this->get_logger(), "Invalid transformation detected!");
        return;
    }
    std::cout << "odom_msg time: " << odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1e9 << std::endl;
    // position_measurement->set_header(msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9, msg->header.frame_id);
    position_measurement->set_header(odom_msg->header.stamp.sec, 
        odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1e9, 
        odom_msg->header.frame_id);
    position_measurement->set_translation(transformed_translation);
    position_measurement->set_transformation();
    std::cout << "pose after transform: " << transformed_translation << std::endl;
    std::cout << "odom_msg time: " << odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec / 1e9 << std::endl;
    // std::lock_guard<std::mutex> lock(*mutex);
    position_mutex.get()->lock();
    std::cout << "position_measurement...trans: " << position_measurement->get_transformation()<< std::endl;
    position_queue->push(position_measurement);
    position_mutex.get()->unlock();
    // RCLCPP_INFO(this->get_logger(), "Odom measurement added to queue. Queue size: %lu", position_queue->size());
}


void ROSSubscriber::GPS2PositionCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    std::shared_ptr<std::mutex> position_mutex,
    OdomQueuePtr position_queue,
    Eigen::Vector3d& reference_position)
{
    // If somehow called before reference initialized, skip safely
    if (!reference_initialized) {
        RCLCPP_WARN(node_->get_logger(), "Reference position not initialized yet! Ignoring GPS message.");
        return;
    }

    auto position_measurement = std::make_shared<OdomMeasurement>();

    double lat0 = reference_position(0);
    double lon0 = reference_position(1);
    double alt0 = reference_position(2);

    // Convert GPS to ENU
    measurement::NavSatMeasurement<double> navsat_measurement;
    navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    Eigen::Matrix<double, 3, 1> enu_translation = navsat_measurement.get_enu(lat0, lon0, alt0);

    // Feed the raw GPS antenna position to the invariant correction. The
    // antenna lever arm is modeled in PositionCorrection as z = p + R*r.
    position_measurement->set_header(
        gps_msg->header.stamp.sec,
        gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec / 1e9,
        gps_msg->header.frame_id);

    position_measurement->set_translation(enu_translation);
    position_measurement->set_transformation();

    // Push into queue with thread safety
    {
        std::lock_guard<std::mutex> lock(*position_mutex);
        position_queue->push(position_measurement);
    }
}

void ROSSubscriber::GPSIMU2PoseCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg,
    std::shared_ptr<std::mutex> pose_mutex,
    const Eigen::Quaterniond& imu_orientation,
    OdomQueuePtr pose_queue,
    Eigen::Vector3d& reference_position)
{
    if (!reference_initialized) {
        RCLCPP_WARN(node_->get_logger(), "Reference position not initialized yet! Ignoring GPS message.");
        return;
    }

    auto pose_measurement = std::make_shared<OdomMeasurement>();

    // Extract reference position
    double lat0 = reference_position(0);
    double lon0 = reference_position(1);
    double alt0 = reference_position(2);

    // Convert GPS to ENU
    measurement::NavSatMeasurement<double> navsat_measurement;
    navsat_measurement.set_navsatfix(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    Eigen::Vector3d enu_translation = navsat_measurement.get_enu(lat0, lon0, alt0);

    const Eigen::Vector3d body_to_gps =
        gps_src_to_body_.block<3, 1>(0, 3);
    const Eigen::Vector3d body_position =
        enu_translation -
        imu_orientation.toRotationMatrix() * body_to_gps;

    // Compose final pose (position + orientation)
    pose_measurement->set_header(
        gps_msg->header.stamp.sec,
        gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec / 1e9,
        gps_msg->header.frame_id);

    pose_measurement->set_translation(body_position);
    pose_measurement->set_rotation(imu_orientation);

    pose_measurement->set_transformation();

    // Add to queue with thread safety
    {
        std::lock_guard<std::mutex> lock(*pose_mutex);
        pose_queue->push(pose_measurement);
    }
}



void ROSSubscriber::RosSpin() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();
}
} // namespace ros_wrapper
