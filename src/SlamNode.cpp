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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include "IOWrapper/Pangolin/PangolinSOSVIOViewer.h"

#include "SlamNode.h"

using namespace dso;

SlamNode::SlamNode(int start_frame, double td_cam_imu,
                   const std::vector<double> &tfm_cam1_cam0,
                   const std::string &calib0, const std::string &calib1,
                   const std::string &vignette0, const std::string &vignette1,
                   const std::string &gamma0, const std::string &gamma1)
    : startFrame(start_frame), tdCamImu(td_cam_imu),
      tfmCam1Cam0(tfm_cam1_cam0) {
  undistorter0 = Undistort::getUndistorterForFile(calib0, gamma0, vignette0);
  if (setting_enable_scale_opt) {
    undistorter1 = Undistort::getUndistorterForFile(calib1, gamma1, vignette1);
    assert((int)undistorter0->getSize()[0] == (int)undistorter1->getSize()[0]);
    assert((int)undistorter0->getSize()[1] == (int)undistorter1->getSize()[1]);
  }

  setGlobalCalib((int)undistorter0->getSize()[0],
                 (int)undistorter0->getSize()[1],
                 undistorter0->getK().cast<float>());

  Mat33f K1 = Mat33f::Zero();
  if (setting_enable_scale_opt) {
    K1 = undistorter1->getK().cast<float>();
  }
  fullSystem = new FullSystem(tfmCam1Cam0, K1);
  if (undistorter0->photometricUndist != 0)
    fullSystem->setGammaFunction(undistorter0->photometricUndist->getG());

  IOWrap::PangolinSOSVIOViewer *pangolinViewer = 0;
  if (!disableAllDisplay) {
    pangolinViewer = new IOWrap::PangolinSOSVIOViewer(wG[0], hG[0], true);
    fullSystem->outputWrapper.push_back(pangolinViewer);
  }
  // setLoopHandler is called even if loop closure is disabled
  // because results are recordered by LoopHandler
  // but loop closure is disabled internally if loop closure is disabled
  IOWrap::LoopHandler *loopHandler = new IOWrap::LoopHandler(pangolinViewer);
  fullSystem->outputWrapper.push_back(loopHandler);

  incomingId = 0;
  isLost = false;
}

SlamNode::~SlamNode() {
  delete undistorter0;
  if (setting_enable_scale_opt) {
    delete undistorter1;
  }
  for (auto &ow : fullSystem->outputWrapper) {
    delete ow;
  }
  delete fullSystem;
}

void SlamNode::imuMessageCallback(const sensor_msgs::ImuConstPtr &msg) {
  boost::unique_lock<boost::mutex> lock(imuQueueMutex);
  Vec7 imu_data;
  imu_data[0] = msg->header.stamp.toSec() - tdCamImu;
  imu_data.segment<3>(1) << msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z;
  imu_data.tail(3) << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  imuQueue.push(imu_data);
}

void SlamNode::imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
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
  cv::Mat img0;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  MinimalImageB minImg0((int)img0.cols, (int)img0.rows,
                        (unsigned char *)img0.data);
  ImageAndExposure *cur_img0 =
      undistorter0->undistort<unsigned char>(&minImg0, 1, 0, 1.0f);
  cur_img0->timestamp = msg0->header.stamp.toSec();

  ImageAndExposure *cur_img1 = nullptr;
  if (setting_enable_scale_opt) {
    cv::Mat img1;
    try {
      img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    MinimalImageB minImg1((int)img1.cols, (int)img1.rows,
                          (unsigned char *)img1.data);
    cur_img1 = undistorter1->undistort<unsigned char>(&minImg1, 1, 0, 1.0f);
    cur_img1->timestamp = msg1->header.stamp.toSec();
  }

  std::vector<Vec7> cur_imu_data;
  if (!setting_enable_imu) {
    process(cur_img0, cur_img1, cur_imu_data);
    delete cur_img0;
    delete cur_img1;
    incomingId++;
    return;
  }

  boost::unique_lock<boost::mutex> imu_lock(imuQueueMutex);
  imgQueue.push({cur_img0, cur_img1});
  while (!imuQueue.empty() && !imgQueue.empty() &&
         imgQueue.front().first->timestamp < imuQueue.back()[0]) {
    // get the current image pair in the front of the queue
    cur_img0 = imgQueue.front().first;
    cur_img1 = imgQueue.front().second;
    imgQueue.pop();

    cur_imu_data.clear();
    // get all imu data by current img timestamp
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

      process(cur_img0, cur_img1, cur_imu_data);
    }
    delete cur_img0;
    delete cur_img1;
    incomingId++;
  }
}

void SlamNode::process(ImageAndExposure *img0, ImageAndExposure *img1,
                       const std::vector<Vec7> &imu_data) {
  fullSystem->addActiveFrame(incomingId, img0, img1, imu_data);

  // reinitialize if necessary
  if (fullSystem->initFailed) {
    auto lastPose = fullSystem->curPose;
    int existing_kf_size = fullSystem->getTotalKFSize();
    std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
    delete fullSystem;

    printf("Reinitializing\n");
    Mat33f K1 = Mat33f::Zero();
    if (setting_enable_scale_opt) {
      K1 = undistorter1->getK().cast<float>();
    }
    fullSystem = new FullSystem(tfmCam1Cam0, K1, existing_kf_size);
    if (undistorter0->photometricUndist != 0)
      fullSystem->setGammaFunction(undistorter0->photometricUndist->getG());
    fullSystem->outputWrapper = wraps;
    fullSystem->curPose = lastPose;
    // setting_fullResetRequested=false;
  }

  if (fullSystem->isLost) {
    printf("LOST!!\n");
    isLost = true;
  }
}
