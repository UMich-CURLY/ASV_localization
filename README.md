# ASV Localization: GPS-Aided Heading Correction for DRIFT

![all_robots](figures/drift_all_robots.gif?raw=true "Title")

## Description

This repository specializes [DRIFT (Dead Reckoning In Field Time)](https://umich-curly.github.io/DRIFT_Website/) — the University of Michigan CURLY Lab's Invariant EKF (InEKF) state estimation library — for GPS-aided heading correction on an autonomous surface vehicle (ASV). DRIFT is a general-purpose, open-source C++ library for proprioceptive state estimation across legged, wheeled, and marine robots; this repo builds on it to fuse single-antenna GPS position fixes with IMU orientation in a single joint InEKF correction, evaluated on field recordings from a Blueboat ASV.

Detailed documentation and tutorials for the underlying DRIFT library can be found at [https://umich-curly.github.io/DRIFT_Website/](https://umich-curly.github.io/DRIFT_Website/).

## Dependencies

Tested on **Ubuntu 22.04** with **ROS 2 Humble**.

> ### C++17 Compiler
Required for the library's threading functionality.

> ### Eigen3
Required by header files. Install instructions: http://eigen.tuxfamily.org. **Requires at least 3.1.0**.

> ### Yaml-cpp
Required by header files. Install instructions: https://github.com/jbeder/yaml-cpp.

> ### ROS 2
The estimator library itself is ROS-independent, but the `blueboat_gps_ros2` node in `ROS2/drift_ros2` requires ROS 2. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) if you don't have it.

## Building

Clone and build both the core `drift` library and the `drift_ros2` wrapper from the repo root:

```bash
git clone https://github.com/UMich-CURLY/ASV_localization.git
cd ASV_localization
source /opt/ros/humble/setup.bash
colcon build --paths . ROS2/drift_ros2
```

## Running

```bash
source install/setup.bash
ros2 run drift_ros2 blueboat_gps_ros2
```

**Consumes:**
- `/imu/data` — orientation (non-AHRS: raw gyroscope + accelerometer, no magnetometer correction)
- `/imu/data_raw` — propagation (raw angular velocity + linear acceleration)
- `/navsatfix` — GPS position

**Publishes:**
- `/localization/pose`
- `/localization/twist`
- `/localization/path`

Tuning lives in `config/blueboat_real/pose_correction.yaml`. 
Additional functional parameters live in `ROS2/drift_ros2/config/blueboat_real/ros_comm.yaml`.

`decouple_prior_covariance` defaults to `true`: before computing the joint correction's Kalman gain, the position–attitude and position–bias blocks of the *prior* covariance are excluded from that one gain computation (the filter's stored covariance is otherwise unaffected).
Set it `false` to run the original, unconditioned joint correction for comparison. Note, that setting when set to `false`, the pure position measurement perturbs attitude despite having no explicit dependence on it.

## Repository layout

```
include/, src/          Core InEKF estimator library (propagation + correction)
ROS2/drift_ros2/         ROS 2 node, topic wiring, and per-platform config
config/                  Filter tuning (noise values, correction settings)
analysis_scripts/        Field-bag evaluation tooling (heading/position/twist RMSE against dual-GNSS)
```

## Citations

This work builds on DRIFT. If you find this repository useful, please cite:

* Tzu-Yuan Lin, Tingjun Li, Wenzhe Tong, and Maani Ghaffari. "Proprioceptive Invariant Robot State Estimation." arXiv preprint arXiv:2311.04320 (2023). (Under review for Transactions on Robotics)
```
@article{lin2023proprioceptive,
  title={Proprioceptive Invariant Robot State Estimation},
  author={Lin, Tzu-Yuan and Li, Tingjun and Tong, Wenzhe and Ghaffari, Maani},
  journal={arXiv preprint arXiv:2311.04320},
  year={2023}
}
```
* Tzu-Yuan Lin, Ray Zhang, Justin Yu, and Maani Ghaffari. "Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events." In Conference on Robot Learning. PMLR, 2021.
```
@inproceedings{lin2021legged,
   title={Legged Robot State Estimation using Invariant Kalman Filtering and Learned Contact Events},
   author={Tzu-Yuan Lin and Ray Zhang and Justin Yu and Maani Ghaffari},
   booktitle={5th Annual Conference on Robot Learning},
   year={2021},
   url={https://openreview.net/forum?id=yt3tDB67lc5}
}
```

## License

Released under a [BSD 3-Clause License](LICENSE).
