#!/bin/bash
#
# ROS2 generates the necessary C++ files for your custom ROS msg/srv.
# Usage: ./gencpp.sh [namespace]

echo "Building custom_sensor_msgs for ROS2"

MSG_NAMESPACE=custom_sensor_msgs
MSG_PATH=./ROS2/drift/msg
MSG_HEADER_OUTPUT_PATH=./ROS2/drift/include/$MSG_NAMESPACE/

# Ensure the include directory exists
mkdir -p $MSG_HEADER_OUTPUT_PATH

# Process each msg and generate code
for file in $MSG_PATH/*.msg
do
  if [[ -f $file ]]
  then
    echo "Processing $file"
    ros2 pkg create --build-type ament_cmake --dependencies std_msgs custom_sensor_msgs

    # Ensure that rosidl generates code for the message
    colcon build --packages-select custom_sensor_msgs
  fi
done

