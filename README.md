# DRIFT: Dead Reckoning In Field Time
![all_robots](figures/drift_all_robots.gif?raw=true "Title")

## Description
Dead Reckoning In Field Time (DRIFT) is an open-source C++ software library designed to provide accurate and high-frequency proprioceptive state estimation. This fork focuses exclusively on the marine robots used in our field experiments and exposes a streamlined ROS 2 interface centered around the WAM-V configurations (`wamv_gps_ros2` and `wamv_gpsimu_ros2`). The core estimator remains modular and extensible, leveraging symmetry-preserving filters such as [Invariant Kalman Filtering (InEKF)](https://www.annualreviews.org/doi/10.1146/annurev-control-060117-105010), but all robot-specific assets outside of the WAM-V marine platforms have been removed to simplify maintenance.

## Framework
![flow_chart](figures/flow_chart.jpg?raw=true "flow chart")

## Run Time Analysis
We perform runtime evaluations using a personal laptop with an Intel i5-11400H CPU and an NVIDIA Jetson AGX Xavier (CPU). DRIFT can operate at an extremely high frequency using CPU-only computation, even on the resourced-constrained Jetson AGX Xavier. For the optional contact estimator, the inference speed on an NVIDIA RTX 3090 GPU is approximately 1100 Hz, and the inference speed on a Jetson AGX Xavier (GPU) is around 830 Hz after TensorRT optimization.

![run_time](figures/run_time.png?raw=true "run time")

# Dependencies
We have tested the library in **Ubuntu 20.04** and **22.04**, but it should be easy to compile in other platforms.

> ### C++17 Compiler
We use the threading functionalities of C++17.


> ### Eigen3
Required by header files. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Requires at least 3.1.0**.

> ### Yaml-cpp
Required by header files. Download and install instructions can be found at: https://github.com/jbeder/yaml-cpp.

> ### ROS 2 (Required for WAM-V use)
ROS 2 Humble (or newer) is required to build and run the provided marine robot nodes. See the ROS 2 section below for build instructions.

# Building DRIFT library

Clone the repository:
```
git clone -b ros2 https://github.com/spsingh37/drift.git
cd drift
```
Create another directory which we will name 'build' and use cmake and make to compile an build project:

```
mkdir build
cd build
cmake ..
make -j4
```

## Install the library
After building the library, you can install the library to the system. This will allow other projects to find the library without needing to specify the path to the library. 

```
sudo make install
```
Then, you can include the library in your project by adding the following line to your CMakeLists.txt file (this is already done in this repo so ignore):
```
find_package(drift REQUIRED)
```

# ROS 2
The `ROS2/drift_ros2` workspace contains everything needed to run the estimator on the supported marine robots:

- `examples/`: entry points for the `wamv_gps_ros2` and `wamv_gpsimu_ros2` nodes.
- `config/`: launch-time YAML files describing the ROS 2 topics and estimator parameters for each WAM-V configuration.
- `src/custom_sensor_msgs`: the ROS 2 messages used by the estimator.

## Building the ROS 2 workspace
You can either run the helper script or issue the equivalent commands yourself.

```
./build_ros.sh
```

The script simply sources the repository root, builds `custom_sensor_msgs`, and then builds the rest of the workspace with `colcon build --symlink-install`. To perform these steps manually:

```
cd ROS2/drift_ros2
colcon build --packages-select custom_sensor_msgs
source install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the WAM-V examples
**WAM-V (GPS + IMU fusion)**
```
ros2 run drift_ros2 wamv_gpsimu_ros2
```

**WAM-V (GPS-only correction)**
```
ros2 run drift_ros2 wamv_gps_ros2
```

Both nodes expose their estimator settings through the YAML files found in `config/wamv_gpsimu_ros2` and `config/wamv_gps_ros2` (and the mirrored ROS 2 communication YAML in `ROS2/drift_ros2/config`). Adjust these files to suit your sensors or deployment environment.

# Contact Estimation
The contact estimation and the contact data set can be found in https://github.com/UMich-CURLY/deep-contact-estimator.

# Citations
If you find this work useful, please kindly cite the following papers

* Tzu-Yuan Lin, Tingjun Li, Wenzhe Tong, and Maani Ghaffari. "Proprioceptive Invariant Robot State Estimation." arXiv preprint arXiv:2311.04320 (2023). (Under review for Transaction on Robotics)
```
@article{lin2023proprioceptive,
  title={Proprioceptive Invariant Robot State Estimation},
  author={Lin, Tzu-Yuan and Li, Tingjun and Tong, Wenzhe and Ghaffari, Maani},
  journal={arXiv preprint arXiv:2311.04320},
  year={2023}
}
```
* Tzu-Yuan Lin, Ray Zhang, Justin Yu, and Maani Ghaffari. "Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events." In Conference on robot learning. PMLR, 2021
```
@inproceedings{
   lin2021legged,
   title={Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events},
   author={Tzu-Yuan Lin and Ray Zhang and Justin Yu and Maani Ghaffari},
   booktitle={5th Annual Conference on Robot Learning },
   year={2021},
   url={https://openreview.net/forum?id=yt3tDB67lc5}
}
```

# License
DRIFT is released under a [BSD 3-Clause License](https://github.com/UMich-CURLY/drift/blob/main/LICENSE). 
