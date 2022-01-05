# Scale Optimized Spline SLAM (SOS-SLAM)

## Related Publications
- **Direct Sparse Odometry**, J. Engel, V. Koltun, D. Cremers, IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI), 2018.
- **Extending Monocular Visual Odometry to Stereo Camera Systems by Scale Optimization**, J. Mo and J. Sattar, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2019.
- **A Fast and Robust Place Recognition Approach for Stereo Visual Odometry Using LiDAR Descriptors**, J. Mo and J. Sattar, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2020.
- **Fast Direct Stereo Visual SLAM**, J. Mo, Md Jahidul Islam, and J. Sattar, IEEE Robotics and Automation Letters, 2022.
- **Continuous-Time Spline Visual-Inertial Odometry**, J. Mo and J. Sattar, under review, [arXiv](https://arxiv.org/abs/2109.09035).

## Dependencies
[ROS](https://www.ros.org/), [PCL](https://pointclouds.org/), [g2o](https://github.com/RainerKuemmerle/g2o), and [DSO dependencies](https://github.com/JakobEngel/dso#21-required-dependencies) ([OpenCV](https://opencv.org/), [Pangolin](https://github.com/stevenlovegrove/Pangolin))

## Install
1. Build g2o library:
```
cd thirdparty/g2o
bash build.sh
```

2. Build SOS-SLAM
```
cd catkin_ws/src
git clone https://github.com/IRVLab/SOS-SLAM.git
cd ..
catkin_make
```

## Usage
### Preparation
- Calibrate stereo cameras with format of [cams](https://github.com/IRVLab/SOS-SLAM/blob/master/cams). T_stereo is the pose of camera0 in camera1 coordinate, rememeber to put a small number in T_stereo[2,2] for numerical stability if images are stereo pre-calibrated. Refer to [DSO](https://github.com/JakobEngel/dso) for more details of intrisic parameters.
- Create a launch file with the format of [sample.launch](https://github.com/IRVLab/SOS-SLAM/blob/master/launch/sample.launch).
### Parameters (in launch file)
- scale_opt_thres: scale optimization accept threshold (e.g., 15.0)
- lidar_range: imitated LiDAR scan range, set to -1 to disable loop closure (e.g., 40.0 meters)
- scan_context_thres: Scan Context threshold for a potential loop closure  (e.g., 0.33)
### Run

```
roslaunch sos_slam [YOUR_LAUNCH_FILE]
```

Ctrl-C to terminate the program, the final trajectory (dslam.txt) will be written to ~/.ros folder.

## Output file (in ~/.ros folder)
- dslam.txt: final trajectory [incoming_id, x, y, z];
- sodso.txt: the trajectory without loop closure, output for comparision.
