# Scale Optimized Spline SLAM (SOS-SLAM)

## Related Publications
- **Direct Sparse Odometry**, J. Engel, V. Koltun, D. Cremers, IEEE Transactions on Pattern Analysis and Machine Intelligence, 2018.
- **Extending Monocular Visual Odometry to Stereo Camera Systems by Scale Optimization**, J. Mo and J. Sattar, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2019.
- **A Fast and Robust Place Recognition Approach for Stereo Visual Odometry Using LiDAR Descriptors**, J. Mo and J. Sattar, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2020.
- **Fast Direct Stereo Visual SLAM**, J. Mo, Md Jahidul Islam, and J. Sattar, IEEE Robotics and Automation Letters, 2022.
- **Continuous-Time Spline Visual-Inertial Odometry**, J. Mo and J. Sattar, IEEE International Conference on Robotics and Automation, 2022.


## Installation
0. Dependencies: [ROS](https://www.ros.org/) (middleware), [DSO dependencies](https://github.com/JakobEngel/dso#21-required-dependencies) ([OpenCV](https://opencv.org/), [Pangolin](https://github.com/stevenlovegrove/Pangolin)), and [PCL](https://pointclouds.org/) (for 3D loop closure).

1. Build g2o library (for global pose optimization when loop closure is enabled):
```
cd thirdparty/g2o
bash build.sh
```

2. Build SOS-SLAM:
```
cd catkin_ws/src
git clone https://github.com/IRVLab/SOS-SLAM.git
cd ..
catkin_make
```

## Usage
1. Preparation
- [Calibrate](https://github.com/ethz-asl/kalibr) your stereo visual-inertial rig and convert to [this format](https://github.com/IRVLab/SOS-SLAM/tree/main/calibs/Gazebo).
    - Refer to [DSO](https://github.com/JakobEngel/dso) for more details of intrisic parameters ([cameraX.txt](https://github.com/IRVLab/SOS-SLAM/blob/main/calibs/Gazebo/camera0.txt)).
    - Put a small number in [T_cam1_cam0[2,2]](https://github.com/IRVLab/SOS-SLAM/blob/main/calibs/Gazebo/calib.yaml#L15) for numerical stability if images are stereo pre-rectified. 
- Create a [launch file](https://github.com/IRVLab/SOS-SLAM/blob/main/launch).
    - **scale_opt_thres**: scale optimization accept threshold (e.g., 15.0), set -1 to disable scale optimization.
    - **weight_imu_dso**: relative weight between inertial and visual systems (e.g., 6.0), set -1 to ignore IMU measurements.
    - **loop_lidar_range**: imitated LiDAR scan range (e.g., 40.0 meters), set -1 to disable place recognition.
    - check ROS parameter acquisition (*getParam()* and *param()*) in [main.cpp](https://github.com/IRVLab/SOS-SLAM/blob/main/src/main.cpp) and [settings.cpp](https://github.com/IRVLab/SOS-SLAM/blob/main/src/util/settings.cpp) for more optional parameters.

2. Run
```
roslaunch sos_slam [YOUR_LAUNCH_FILE]
```

### Outputs
- Published ROS Topics:
    - **pose_cam0_in_world/current**: pose of the most recent frame (not final, minimal lagging).
    - **pose_cam0_in_world/marginalized**: pose of the most recently marginalized frame (final, several keyframes behind).
- **poses.txt** will also be written to ~/.ros for pose evaluation once done.