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

// This file is modified from <https://github.com/JakobEngel/dso>

#pragma once
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include <deque>
#include <map>
#include <pangolin/pangolin.h>

namespace dso {

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap {

class KeyFrameDisplay;

class PangolinSOSVIOViewer : public Output3DWrapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PangolinSOSVIOViewer(int w_, int h_, bool startRunThread = true);
  virtual ~PangolinSOSVIOViewer();

  void run();
  void close();

  void addImageToDisplay(std::string name, MinimalImageB3 *image);
  void clearAllImagesToDisplay();

  // ==================== Output3DWrapper Functionality ======================
  virtual void publishKeyframes(std::vector<FrameHessian *> &frames, bool final,
                                CalibHessian *HCalib) override;

  virtual void modifyKeyframePoseByKFID(int id, const SE3 &poseCamToWorld);

  void refreshLidarData(const std::vector<Eigen::Vector3d> &pts, size_t cur_sz);

  virtual void pushLiveFrame(FrameHessian *image) override;

  virtual void pushDepthImage(MinimalImageB3 *image) override;

  virtual void join() override;

private:
  void drawConstraints();

  void drawLidar();

  boost::thread runThread;
  bool running;
  int w, h;

  // 3D model rendering
  boost::mutex model3dMutex;
  std::vector<size_t> keyframesIdSorted;
  std::map<int, KeyFrameDisplay *> keyframesById;

  // lidar rendering
  boost::mutex modelLidarMutex;
  std::vector<Eigen::Vector3d> lidarPts;
  size_t lidarCurSize;

  // images rendering
  boost::mutex openImagesMutex;
  MinimalImageB3 *internalVideoImg;
  bool videoImgChanged;
  MinimalImageB3 *internalKFImg;
  bool KFImgChanged;
};

} // namespace IOWrap

} // namespace dso
