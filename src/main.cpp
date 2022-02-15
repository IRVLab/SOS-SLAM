// Copyright (C) <2022> <Jiawei Mo, Junaed Sattar>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "SlamNode.h"

using namespace dso;

void settingsDefault(int preset, int mode) {
  printf("=============== PRESET Settings: ===============\n");
  if (preset == 1 || preset == 3) {
    printf("preset=%d is not supported", preset);
    exit(1);
  }
  if (preset == 0) {
    printf("DEFAULT settings:\n"
           "- 2000 active points\n"
           "- 5-7 active frames\n"
           "- 1-6 LM iteration each KF\n"
           "- original image resolution\n");

    setting_desiredImmatureDensity = 1500;
    setting_desiredPointDensity = 2000;
    setting_minFrames = 5;
    setting_maxFrames = 7;
    setting_maxOptIterations = 6;
    setting_minOptIterations = 1;
  }

  if (preset == 2) {
    printf("FAST settings:\n"
           "- 800 active points\n"
           "- 4-6 active frames\n"
           "- 1-4 LM iteration each KF\n"
           "- 424 x 320 image resolution\n");

    setting_desiredImmatureDensity = 600;
    setting_desiredPointDensity = 800;
    setting_minFrames = 4;
    setting_maxFrames = 6;
    setting_maxOptIterations = 4;
    setting_minOptIterations = 1;

    benchmarkSetting_width = 424;
    benchmarkSetting_height = 320;
  }

  printf("IMU: %s\n", setting_enable_imu ? "enabled" : "disabled");
  printf("Scale Optimization: %s\n",
         setting_enable_scale_opt ? "enabled" : "disabled");
  printf("Loop Closure: %s\n",
         setting_enable_loop_closure ? "enabled" : "disabled");

  if (mode == 0) {
    printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
  }
  if (mode == 1) {
    printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
  }
  if (mode == 2) {
    printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
    setting_photometricCalibration = 0;
    setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_minGradHistAdd = 3;
  }

  printf("==============================================\n");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sos_slam");
  ros::NodeHandle nhPriv("~");

  /* **************************** DSO parameters **************************** */
  bool nomt;
  int preset, mode;
  nhPriv.param("quiet", setting_debugout_runquiet, true);
  nhPriv.param("nogui", disableAllDisplay, false);
  nhPriv.param("nomt", nomt, false);
  nhPriv.param("preset", preset, 0);
  nhPriv.param("mode", mode, 1);
  multiThreading = !nomt;

  std::string cam0_topic, calib0, vignette0, gamma0;
  if (!nhPriv.getParam("cam0_topic", cam0_topic) ||
      !nhPriv.getParam("calib0", calib0)) {
    ROS_INFO("Failed to get DSO topics/params!");
    return -1;
  }
  nhPriv.param<std::string>("vignette0", vignette0, "");
  nhPriv.param<std::string>("gamma0", gamma0, "");

  /* **************************** IMU parameters **************************** */
  nhPriv.param("weight_imu_dso", setting_weight_imu_dso, -1.0);
  setting_enable_imu = setting_weight_imu_dso > 0;

  std::string imu_topic;
  std::vector<double> tfm_cam0_imu;
  double imu_rate, imu_acc_nd, imu_acc_rw, imu_gyro_nd, imu_gyro_rw, td_cam_imu;
  if (setting_enable_imu) {
    if (!nhPriv.getParam("imu_topic", imu_topic) ||
        !nhPriv.getParam("T_cam0_imu", tfm_cam0_imu) ||
        !nhPriv.getParam("rate_hz", imu_rate) ||
        !nhPriv.getParam("accelerometer_noise_density", imu_acc_nd) ||
        !nhPriv.getParam("accelerometer_random_walk", imu_acc_rw) ||
        !nhPriv.getParam("gyroscope_noise_density", imu_gyro_nd) ||
        !nhPriv.getParam("gyroscope_random_walk", imu_gyro_rw)) {
      ROS_INFO("IMU is enabled but failed to get topics/params!");
      return -1;
    }

    Eigen::Matrix<double, 16, 1> tfm_cam0_imu_vec(tfm_cam0_imu.data());
    Eigen::Map<Mat44> tfm_cam0_imu(tfm_cam0_imu_vec.data(), 4, 4);
    tfm_cam0_imu.transposeInPlace(); // transposed Mat44 initialization
    setting_rot_imu_cam = tfm_cam0_imu.eval().topLeftCorner<3, 3>().transpose();

    setting_weight_imu = Mat66::Identity();
    setting_weight_imu.topLeftCorner<3, 3>() /=
        (imu_acc_nd * imu_acc_nd * imu_rate);
    setting_weight_imu.bottomRightCorner<3, 3>() /=
        (imu_gyro_nd * imu_gyro_nd * imu_rate);
    setting_weight_imu *= setting_weight_imu_dso;

    setting_weight_imu_bias = Mat66::Identity();
    setting_weight_imu_bias.topLeftCorner<3, 3>() /= (imu_acc_rw * imu_acc_rw);
    setting_weight_imu_bias.bottomRightCorner<3, 3>() /=
        (imu_gyro_rw * imu_gyro_rw);
    setting_weight_imu_bias *= setting_weight_imu_dso;
  }
  nhPriv.param("timeshift_cam_imu", td_cam_imu, 0.0);
  setting_gravity << 0, 0, -9.81;

  /* *********************** stereo camera parameters *********************** */
  nhPriv.param("scale_opt_thres", setting_scale_opt_thres, -1.0f);
  setting_enable_scale_opt = setting_scale_opt_thres > 0;

  std::vector<double> tfm_cam1_cam0;
  std::string cam1_topic, calib1, vignette1, gamma1;
  if (setting_enable_scale_opt) {
    if (!nhPriv.getParam("T_cam1_cam0", tfm_cam1_cam0) ||
        !nhPriv.getParam("cam1_topic", cam1_topic) ||
        !nhPriv.getParam("calib1", calib1)) {
      ROS_INFO("Stereo mode is enabled but failed to get topics/params!");
      return -1;
    }
  }
  nhPriv.param<std::string>("vignette1", vignette1, "");
  nhPriv.param<std::string>("gamma1", gamma1, "");

  /* *********************** loop closure parameters ************************ */
  std::string cam_mode;
  nhPriv.param<std::string>("loop_cam_mode", cam_mode, "downward");
  setting_cam_mode = (cam_mode == "downward") ? DOWNWARD_CAM : FORWARD_CAM;
  nhPriv.param("loop_lidar_range", setting_lidar_range, -1.0f);
  nhPriv.param("scan_context_thres", setting_scan_context_thres, 0.1f);
  setting_enable_loop_closure = setting_lidar_range > 0;
  if (setting_enable_loop_closure) {
    if (!(setting_enable_imu || setting_enable_scale_opt)) {
      ROS_INFO("Loop closure for monocular VO is not implemented!");
      return -1;
    }
  }
  nhPriv.param("loop_direc_thres", setting_loop_direct_thres, 10.0f);
  nhPriv.param("loop_force_icp", setting_loop_force_icp, false);
  nhPriv.param("loop_icp_thres", setting_loop_icp_thres, 1.5f);

  /* ******************* read from a bag file [optional] ******************** */
  std::string bag_path;
  int start_frame;
  nhPriv.param<std::string>("bag", bag_path, "");
  nhPriv.param("start_frame", start_frame, 0);

  /* ************************* create the slam node ************************* */
  settingsDefault(preset, mode);
  SlamNode *slam_node =
      new SlamNode(start_frame, td_cam_imu, tfm_cam1_cam0, calib0, calib1,
                   vignette0, vignette1, gamma0, gamma1);

  if (!bag_path.empty()) {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics = {imu_topic, cam0_topic, cam1_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::ImageConstPtr img0, img1;
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
      if (slam_node->isLost) {
        break;
      }
      if (m.getTopic() == imu_topic) {
        sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        slam_node->imuMessageCallback(imu);
      }
      if (m.getTopic() == cam0_topic) {
        img0 = m.instantiate<sensor_msgs::Image>();
      }
      if (m.getTopic() == cam1_topic) {
        img1 = m.instantiate<sensor_msgs::Image>();
      }
      if (img0 && (!setting_enable_scale_opt ||
                   (img1 && fabs(img0->header.stamp.toSec() -
                                 img1->header.stamp.toSec()) < 0.1))) {
        slam_node->imageMessageCallback(img0, img1);
        img0 = nullptr;
        img1 = nullptr;
      }
    }
    bag.close();
  } else {
    ros::NodeHandle nh;

    // ROS subscribe to imu data
    ros::Subscriber imu_sub = nh.subscribe(
        imu_topic, 10000, &SlamNode::imuMessageCallback, slam_node);

    // ROS subscribe to stereo images
    auto *cam0_sub = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, cam0_topic, 10000);
    auto *cam1_sub = new message_filters::Subscriber<sensor_msgs::Image>(
        nh, cam1_topic, 10000);
    auto *sync = new message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>(10),
        *cam0_sub, *cam1_sub);
    sync->registerCallback(
        boost::bind(&SlamNode::imageMessageCallback, slam_node, _1, _2));

    ros::spin();
  }

  delete slam_node;
  ros::spinOnce();
  return 0;
}
