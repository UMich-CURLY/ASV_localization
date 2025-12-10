# Drift ROS2 (ASV)

ROS 2 Drift InEKF estimator for ASV use. The node subscribes to IMU + GPS, feeds them into the Drift core, and publishes fused pose/path/twist.

## What’s in the pipeline
- `examples/wamv_gpsimu_ros2.cpp`: entrypoint that wires IMU+GPS into Drift with pose correction (heading offset supported).
- `examples/wamv_gps_ros2.cpp`: entrypoint that uses IMU propagation and GPS position-only correction.
- `src/communication/ros2_subscriber.cpp`: converts ROS topics to Drift measurement queues; applies GPS->body transform and optional heading offset.
- `src/communication/ros2_publisher.cpp`: publishes fused `PoseStamped`, `Path`, and `TwistStamped`.

### How to modify?
- Per-vehicle  input-output and transforms: `config/wamv_gpsimu_ros2/ros_comm.yaml` or `config/wamv_gps_ros2/ros_comm.yaml` (topics, GPS->body translation/quaternion, heading offset, publish rates).
- Noise/bias tuning: `config/wamv_gpsimu_ros2/imu_propagation.yaml`, `pose_correction.yaml`, `position_correction.yaml` - adjust process/measurement noise here rather than inside the code.

## Build the Drift core 
```bash
git clone
cd drift
mkdir build && cd build
cmake ..
make -j4
```

## Build the ROS2 wrapper
```bash
cd .../drift/ROS2
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select drift_ros2
source install/setup.bash
```

## Configure topics and frames
Edit one of:
- `config/wamv_gpsimu_ros2/ros_comm.yaml` (GPS + IMU) - pose correction
- `config/wamv_gps_ros2/ros_comm.yaml` (GPS position only) - only GPS position correction

Fields:
- `subscribers.imu_topic` / `gps_topic`: upstream ROS topics.
- `translation_gps_source_to_body`, `rotation_gps_source_to_body` ([w,x,y,z]): antenna-to-body transform.
- `heading_offset_deg` (GPS+IMU): extra yaw applied after the antenna rotation.
- `pose_publish_rate`, `path_publish_rate`, and output topics (`pose_publish_topic`, `path_publish_topic`, `twist_publish_topic`).

The first GPS fix seeds the local ENU origin.

## Run
- GPS + IMU pose correction:
  ```bash
  ros2 run drift_ros2 wamv_gpsimu_ros2
  ```
- GPS position-only correction:
  ```bash
  ros2 run drift_ros2 wamv_gps_ros2
  ```

Ensure your sensors are publishing to the topics defined in the YAML.

## Citation
If you use this work, please cite:
```
@article{lin2023proprioceptive,
  title={Proprioceptive Invariant Robot State Estimation},
  author={Lin, Tzu-Yuan and Li, Tingjun and Tong, Wenzhe and Ghaffari, Maani},
  journal={arXiv preprint arXiv:2311.04320},
  year={2023}
}
```
