cmake_minimum_required(VERSION 3.14)
project(drift_ros2)

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(CMAKE_CXX_STANDARD 17)

find_library(LIBRARY_NAME PATHS "/usr/lib/x86_64-linux-gnu" NO_DEFAULT_PATH)


# Ensure dependencies are available
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer chrono)
find_package(yaml-cpp REQUIRED)
find_package(novatel_oem7_msgs REQUIRED) # ADDED FOR HRI

message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Drift include dir: ${drift_INCLUDE_DIRS}")
message("Yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/../../include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${drift_INCLUDE_DIRS}
)

set(LIBS
  ${EIGEN3_LIBS}
  ${Boost_LIBRARIES}
  ${Boost_GENERAL}
)



# Add Drift library
add_library(drift SHARED
  ${PROJECT_SOURCE_DIR}/../../src/state/robot_state.cpp
  ${PROJECT_SOURCE_DIR}/../../src/estimator/inekf_estimator.cpp
  ${PROJECT_SOURCE_DIR}/../../src/math/lie_group.cpp
  ${PROJECT_SOURCE_DIR}/../../src/math/se_k_3.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/base_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/position_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/correction/pose_correction.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/../../src/filter/inekf/inekf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/imu_filter/imu_ang_vel_ekf.cpp
  ${PROJECT_SOURCE_DIR}/../../src/measurement/measurement.cpp
)

target_link_libraries(drift yaml-cpp ${LIBS})

# ROS 2 Communication Library
add_library(ros_communication_lib SHARED
  ${PROJECT_SOURCE_DIR}/src/communication/ros2_subscriber.cpp
  ${PROJECT_SOURCE_DIR}/src/communication/ros2_publisher.cpp
)


target_link_libraries(ros_communication_lib drift)
ament_target_dependencies(ros_communication_lib rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs message_filters novatel_oem7_msgs)

# Add executable targets
set(EXECUTABLES wamv_gps_ros2 wamv_gpsimu_ros2)

foreach(exec ${EXECUTABLES})
  add_executable(${exec} ${PROJECT_SOURCE_DIR}/examples/${exec}.cpp)
  target_link_libraries(${exec} drift ros_communication_lib yaml-cpp ${LIBS})
  ament_target_dependencies(${exec} rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs message_filters)
  install(TARGETS ${exec} DESTINATION lib/${PROJECT_NAME})
endforeach()

# Export dependencies
ament_export_dependencies(rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs message_filters yaml-cpp)

# Finalize package
ament_package()
