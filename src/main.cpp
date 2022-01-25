// Copyright (C) <2020> <Jiawei Mo, Junaed Sattar>

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

#include <chrono>
#include <queue>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "FullSystem/FullSystem.h"
#include "IOWrapper/Pangolin/PangolinSOSVIOViewer.h"
#include "util/Undistort.h"

using namespace dso;

class VioNode {
private:
  int startFrame;
  double tdCamImu;
  int incomingId;
  FullSystem *fullSystem;
  Undistort *undistorter0;
  Undistort *undistorter1;
  std::queue<Vec7> imuQueue;
  std::queue<std::pair<ImageAndExposure *, ImageAndExposure *>> imgQueue;
  boost::mutex imuQueueMutex;
  boost::mutex imgQueueMutex;

  // scale optimizer
  std::vector<double> tfmStereo;

  void settingsDefault(int preset, int mode);

public:
  bool isLost;
  std::vector<double> ttFrame;

  VioNode(int start_frame, double td_cam_imu,
          const std::vector<double> &tfm_stereo, const std::string &calib0,
          const std::string &calib1, const std::string &vignette0,
          const std::string &vignette1, const std::string &gamma0,
          const std::string &gamma1, bool nomt, int preset, int mode);
  ~VioNode();

  void imuMessageCallback(const sensor_msgs::ImuConstPtr &msg);
  void imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
                            const sensor_msgs::ImageConstPtr &msg1);
  void printResult(std::string file) { fullSystem->printResult(file); }
};

void VioNode::settingsDefault(int preset, int mode) {
  printf("\n=============== PRESET Settings: ===============\n");
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

  isLost = false;
}

VioNode::VioNode(int start_frame, double td_cam_imu,
                 const std::vector<double> &tfm_stereo,
                 const std::string &calib0, const std::string &calib1,
                 const std::string &vignette0, const std::string &vignette1,
                 const std::string &gamma0, const std::string &gamma1,
                 bool nomt, int preset, int mode)
    : startFrame(start_frame), tfmStereo(tfm_stereo) {

  // DSO front end
  settingsDefault(preset, mode);

  multiThreading = !nomt;

  undistorter0 = Undistort::getUndistorterForFile(calib0, gamma0, vignette0);
  undistorter1 = Undistort::getUndistorterForFile(calib1, gamma1, vignette1);
  assert((int)undistorter0->getSize()[0] == (int)undistorter1->getSize()[0]);
  assert((int)undistorter0->getSize()[1] == (int)undistorter1->getSize()[1]);

  setGlobalCalib((int)undistorter0->getSize()[0],
                 (int)undistorter0->getSize()[1],
                 undistorter0->getK().cast<float>());

  fullSystem = new FullSystem(tfmStereo, undistorter1->getK().cast<float>());
  if (undistorter0->photometricUndist != 0)
    fullSystem->setGammaFunction(undistorter0->photometricUndist->getG());

  IOWrap::PangolinSOSVIOViewer *pangolinViewer = 0;
  if (!disableAllDisplay) {
    IOWrap::PangolinSOSVIOViewer *pangolinViewer =
        new IOWrap::PangolinSOSVIOViewer(wG[0], hG[0], true);
    fullSystem->outputWrapper.push_back(pangolinViewer);
  }
  // setLoopHandler is called even if loop closure is disabled
  // because results are recordered by LoopHandler
  // but loop closure is disabled internally if loop closure is disabled
  IOWrap::LoopHandler *loopHandler = new IOWrap::LoopHandler(pangolinViewer);
  fullSystem->outputWrapper.push_back(loopHandler);

  incomingId = 0;
}

VioNode::~VioNode() {
  delete undistorter0;
  delete undistorter1;
  for (auto &ow : fullSystem->outputWrapper) {
    delete ow;
  }
  delete fullSystem;
}

void VioNode::imuMessageCallback(const sensor_msgs::ImuConstPtr &msg) {
  boost::unique_lock<boost::mutex> lock(imuQueueMutex);
  Vec7 imu_data;
  imu_data[0] = msg->header.stamp.toSec() - tdCamImu;
  imu_data.segment<3>(1) << msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z;
  imu_data.tail(3) << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  imuQueue.push(imu_data);
}

void VioNode::imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
                                   const sensor_msgs::ImageConstPtr &msg1) {
  if (startFrame > 0) {
    startFrame--;
    incomingId++;
    while (!imuQueue.empty()) {
      imuQueue.pop();
    }
    return;
  }

  boost::unique_lock<boost::mutex> img_lock(imgQueueMutex);
  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  MinimalImageB minImg0((int)img0.cols, (int)img0.rows,
                        (unsigned char *)img0.data);
  ImageAndExposure *undistImg0 =
      undistorter0->undistort<unsigned char>(&minImg0, 1, 0, 1.0f);
  undistImg0->timestamp = msg0->header.stamp.toSec();
  MinimalImageB minImg1((int)img1.cols, (int)img1.rows,
                        (unsigned char *)img1.data);
  ImageAndExposure *undistImg1 =
      undistorter1->undistort<unsigned char>(&minImg1, 1, 0, 1.0f);

  imgQueue.push({undistImg0, undistImg1});

  boost::unique_lock<boost::mutex> imu_lock(imuQueueMutex);
  while (!imuQueue.empty() && !imgQueue.empty() &&
         imgQueue.front().first->timestamp < imuQueue.back()[0]) {
    // current image pair
    ImageAndExposure *cur_img0 = imgQueue.front().first;
    ImageAndExposure *cur_img1 = imgQueue.front().second;
    imgQueue.pop();

    // get all imu data by current img timestamp
    std::vector<Vec7> cur_imu_data;
    while (imuQueue.front()[0] < cur_img0->timestamp) {
      cur_imu_data.push_back(imuQueue.front());
      imuQueue.pop();
    }
    assert(!imuQueue.empty());

    if (!cur_imu_data.empty()) {
      // interpolate imu data at cur image time
      Vec7 last_imu_data =
          ((imuQueue.front()[0] - cur_img0->timestamp) * cur_imu_data.back() +
           (cur_img0->timestamp - cur_imu_data.back()[0]) * imuQueue.front()) /
          ((imuQueue.front()[0] - cur_imu_data.back()[0]));
      last_imu_data[0] = cur_img0->timestamp;
      cur_imu_data.push_back(last_imu_data);

      auto start = std::chrono::steady_clock::now();
      fullSystem->addActiveFrame(cur_imu_data, cur_img0, cur_img1, incomingId);
      auto end = std::chrono::steady_clock::now();
      ttFrame.push_back(
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count());

      // reinitialize if necessary
      if (fullSystem->initFailed) {
        auto lastPose = fullSystem->curPose;
        int existing_kf_size = fullSystem->getTotalKFSize();
        std::vector<IOWrap::Output3DWrapper *> wraps =
            fullSystem->outputWrapper;
        delete fullSystem;

        printf("Reinitializing\n");
        fullSystem = new FullSystem(
            tfmStereo, undistorter1->getK().cast<float>(), existing_kf_size);
        if (undistorter0->photometricUndist != 0)
          fullSystem->setGammaFunction(undistorter0->photometricUndist->getG());
        fullSystem->outputWrapper = wraps;
        fullSystem->curPose = lastPose;
        // setting_fullResetRequested=false;
      }
    }

    delete cur_img0;
    delete cur_img1;
    incomingId++;

    if (fullSystem->isLost) {
      printf("LOST!!\n");
      isLost = true;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sos_slam");
  ros::NodeHandle nhPriv("~");

  /* *********************** required parameters ************************ */
  // stereo camera parameters
  std::vector<double> tfm_imu, tfm_stereo;
  double imu_rate, imu_acc_nd, imu_acc_rw, imu_gyro_nd, imu_gyro_rw;
  std::string imu_topic, cam0_topic, cam1_topic, calib0, calib1, results_path;
  if (!nhPriv.getParam("T_imu/data", tfm_imu) ||
      !nhPriv.getParam("rate_hz", imu_rate) ||
      !nhPriv.getParam("accelerometer_noise_density", imu_acc_nd) ||
      !nhPriv.getParam("accelerometer_random_walk", imu_acc_rw) ||
      !nhPriv.getParam("gyroscope_noise_density", imu_gyro_nd) ||
      !nhPriv.getParam("gyroscope_random_walk", imu_gyro_rw) ||
      !nhPriv.getParam("T_stereo/data", tfm_stereo) ||
      !nhPriv.getParam("imu_topic", imu_topic) ||
      !nhPriv.getParam("cam0_topic", cam0_topic) ||
      !nhPriv.getParam("cam1_topic", cam1_topic) ||
      !nhPriv.getParam("calib0", calib0) ||
      !nhPriv.getParam("calib1", calib1) ||
      !nhPriv.getParam("results", results_path)) {
    ROS_INFO("Fail to get sensor topics/params, exit.!!!!");
    return -1;
  }
  std::string vignette0, vignette1, gamma0, gamma1;
  nhPriv.param<std::string>("vignette0", vignette0, "");
  nhPriv.param<std::string>("vignette1", vignette1, "");
  nhPriv.param<std::string>("gamma0", gamma0, "");
  nhPriv.param<std::string>("gamma1", gamma1, "");

  /* *********************** optional parameters ************************ */
  // DSO settings
  bool nomt;
  int preset, mode;
  std::string vignette, gamma;
  nhPriv.param("quiet", setting_debugout_runquiet, true);
  nhPriv.param("nogui", disableAllDisplay, false);
  nhPriv.param("nomt", nomt, false);
  nhPriv.param("preset", preset, 0);
  nhPriv.param("mode", mode, 1);
  nhPriv.param<std::string>("vignette", vignette, "");
  nhPriv.param<std::string>("gamma", gamma, "");

  double td_cam_imu, g_norm;
  nhPriv.param("timeshift_cam_imu", td_cam_imu, 0.0);
  nhPriv.param("weight_imu_dso", setting_weight_imu_dso, 1.0);
  nhPriv.param("g_norm", g_norm, 9.80665);
  setting_enable_imu = setting_weight_imu_dso > 0;
  setting_gravity << 0, 0, -g_norm;

  // scale optimization threshold
  nhPriv.param("scale_opt_thres", setting_scale_opt_thres, 10.0f);
  if (setting_scale_opt_thres > 0) {
    setting_estimate_scale = false;
  }

  // loop closure parameters
  nhPriv.param("lidar_range", setting_lidar_range, 40.0f);
  nhPriv.param("scan_context_thres", setting_scan_context_thres, 0.33f);

  // read from a bag file
  std::string bag_path;
  int start_frame;
  nhPriv.param<std::string>("bag", bag_path, "");
  nhPriv.param("start_frame", start_frame, 0);

  /* ******************************************************************** */

  VioNode vio_node(start_frame, td_cam_imu, tfm_stereo, calib0, calib1,
                   vignette0, vignette1, gamma0, gamma1, nomt, preset, mode);

  cv::Mat tfm_imu_cv = cv::Mat(tfm_imu);
  tfm_imu_cv = tfm_imu_cv.reshape(0, 4);
  Mat44 tfm_imu_cam;
  cv::cv2eigen(tfm_imu_cv, tfm_imu_cam);
  setting_rot_imu_cam = tfm_imu_cam.topLeftCorner<3, 3>();

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

  if (!bag_path.empty()) {

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics = {imu_topic, cam0_topic, cam1_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    sensor_msgs::ImageConstPtr img0, img1;
    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
      if (vio_node.isLost) {
        break;
      }
      if (m.getTopic() == imu_topic) {
        sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        vio_node.imuMessageCallback(imu);
      }
      if (m.getTopic() == cam0_topic) {
        img0 = m.instantiate<sensor_msgs::Image>();
      }
      if (m.getTopic() == cam1_topic) {
        img1 = m.instantiate<sensor_msgs::Image>();
      }
      if (img0 && img1 &&
          fabs(img0->header.stamp.toSec() - img1->header.stamp.toSec()) < 0.1) {
        vio_node.imageMessageCallback(img0, img1);
        img0 = nullptr;
        img1 = nullptr;
      }
    }
    bag.close();
  } else {
    ros::NodeHandle nh;

    // ROS subscribe to imu data
    ros::Subscriber imu_sub =
        nh.subscribe(imu_topic, 10000, &VioNode::imuMessageCallback, &vio_node);

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
        boost::bind(&VioNode::imageMessageCallback, &vio_node, _1, _2));

    ros::spin();
  }

  vio_node.printResult(results_path);

  int total_frame_tt = 0;
  for (int tt : vio_node.ttFrame) {
    total_frame_tt += tt;
  }
  printf("frame_tt: %.1f\n", float(total_frame_tt) / vio_node.ttFrame.size());

  ros::spinOnce();
  return 0;
}
