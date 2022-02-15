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

#include <queue>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"

using namespace dso;

class SlamNode {
private:
  int startFrame;
  int incomingId;
  FullSystem *fullSystem;
  Undistort *undistorter0;
  std::queue<std::pair<ImageAndExposure *, ImageAndExposure *>> imgQueue;
  boost::mutex imgQueueMutex;

  // scale optimization
  std::vector<double> tfmCam1Cam0;
  Undistort *undistorter1;

  // IMU
  double tdCamImu;
  std::queue<Vec7> imuQueue;
  boost::mutex imuQueueMutex;

public:
  bool isLost;
  SlamNode(int start_frame, double td_cam_imu,
           const std::vector<double> &tfm_cam1_cam0, const std::string &calib0,
           const std::string &calib1, const std::string &vignette0,
           const std::string &vignette1, const std::string &gamma0,
           const std::string &gamma1);
  ~SlamNode();

  void imuMessageCallback(const sensor_msgs::ImuConstPtr &msg);

  void imageMessageCallback(const sensor_msgs::ImageConstPtr &msg0,
                            const sensor_msgs::ImageConstPtr &msg1);

  void process(ImageAndExposure *img0, ImageAndExposure *img1,
               const std::vector<Vec7> &imu_data);
};
