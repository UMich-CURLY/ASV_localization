# DRIFT Marine ROS 2 Wrapper

## Description
Dead Reckoning In Field Time (DRIFT) is a symmetry-preserving proprioceptive state estimation framework built around the Invariant Extended Kalman Filter (InEKF). This trimmed documentation focuses entirely on the WAM-V marine robots that we currently operate and describes the ROS 2 wrapper that exposes the estimator to those platforms.

## Dependencies
- **C++17 compiler** with threading support.
- **Eigen 3.1+**, **yaml-cpp**, and **Boost** for the core estimator.
- **ROS 2 Humble (or newer)** to build and run the WAM-V nodes.

## Building the core library
```
git clone -b ros2 https://github.com/spsingh37/drift.git
cd drift
mkdir build && cd build
cmake ..
make -j4
sudo make install   # optional
```

## ROS 2 usage
The ROS 2 workspace lives in `ROS2/drift_ros2`. Build everything (including the custom message package) via:

```
cd ROS2/drift_ros2
colcon build --packages-select custom_sensor_msgs
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch one of the supported nodes:

```
ros2 run drift_ros2 wamv_gpsimu_ros2   # GPS+IMU fusion
ros2 run drift_ros2 wamv_gps_ros2      # GPS-only correction
```

## Configuration
Only the marine robot assets are retained. All estimator parameters, noise definitions, and topic mappings live under:

- `config/wamv_gpsimu_ros2` (core estimator YAML files)
- `config/wamv_gps_ros2`
- `ROS2/drift_ros2/config/wamv_gpsimu_ros2` (ROS 2 communication topics)
- `ROS2/drift_ros2/config/wamv_gps_ros2`

Adjust these files to match your sensors or deployment site.

## Citations
If you use DRIFT, please cite:

```
@article{lin2023proprioceptive,
  title={Proprioceptive Invariant Robot State Estimation},
  author={Lin, Tzu-Yuan and Li, Tingjun and Tong, Wenzhe and Ghaffari, Maani},
  journal={arXiv preprint arXiv:2311.04320},
  year={2023}
}

@inproceedings{lin2021legged,
  title={Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events},
  author={Lin, Tzu-Yuan and Zhang, Ray and Yu, Justin and Ghaffari, Maani},
  booktitle={5th Annual Conference on Robot Learning},
  year={2021}
}
```

## License
Released under the BSD 3-Clause License.
