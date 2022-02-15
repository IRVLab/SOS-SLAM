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

// This file is modified from <https://github.com/JakobEngel/dso>

#pragma once

#undef Success
#include "util/NumType.h"
#include <Eigen/Core>
#include <pangolin/pangolin.h>

#include <fstream>
#include <sstream>

namespace dso {
class CalibHessian;
class FrameHessian;
class FrameShell;

namespace IOWrap {

template <int ppp> struct InputPointSparse {
  float u;
  float v;
  float idepth;
  float idepth_hessian;
  float relObsBaseline;
  int numGoodRes;
  unsigned char color[ppp];
  unsigned char status;
};

struct MyVertex {
  float point[3];
  unsigned char color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  KeyFrameDisplay();
  ~KeyFrameDisplay();

  // copies points from KF over to internal buffer,
  // keeping some additional information so we can render it differently.
  void setFromKF(FrameHessian *fh, CalibHessian *HCalib);

  // copies & filters internal data to GL buffer for rendering. if nothing to
  // do: does nothing.
  void refreshPC();

  // renders cam & pointcloud.
  void drawCam(float lineWidth = 1, float *color = 0, float sizeFactor = 1);
  void drawPC(float pointSize);

  int id;
  bool active;
  SE3 tfmCToW;
  bool needRefresh;

  inline bool operator<(const KeyFrameDisplay &other) const {
    return (id < other.id);
  }

private:
  float fx, fy, cx, cy;
  float fxi, fyi, cxi, cyi;
  int width, height;

  float myScaledTh, myAbsTh, myScale;
  int mySparsifyFactor;
  int myDisplayMode;
  float myMinRelBs;

  int numSparsePoints;
  int numSparseBufferSize;
  InputPointSparse<MAX_RES_PER_POINT> *originalInputSparse;

  bool bufferValid;
  int numGlBufferPoints;
  int numGlBufferGoodPoints;
  pangolin::GlBuffer vertexBuffer;
  pangolin::GlBuffer colorBuffer;
};

} // namespace IOWrap
} // namespace dso
