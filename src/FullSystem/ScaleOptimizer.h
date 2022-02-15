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

#include "IOWrapper/Output3DWrapper.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "OptimizationBackend/ScaleAccumulator.h"
#include "util/NumType.h"
#include "util/settings.h"

#include <math.h>
#include <vector>

namespace dso {
struct CalibHessian;
struct FrameHessian;
struct PointFrameResidual;

template <int b, typename T>
T *allocAligned(int size, std::vector<T *> &rawPtrVec) {
  const int padT = 1 + ((1 << b) / sizeof(T));
  T *ptr = new T[size + padT];
  rawPtrVec.push_back(ptr);
  T *alignedPtr = (T *)((((uintptr_t)(ptr + padT)) >> b) << b);
  return alignedPtr;
}

class ScaleOptimizer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ScaleOptimizer(int w, int h, const std::vector<double> &tfm_cam1_cam0,
                 const Mat33f &K1);
  ~ScaleOptimizer();

  void makeK(CalibHessian *HCalib);

  float optimizeScale(FrameHessian *fh1, float &scale, int coarsestLvl);

protected:
  std::vector<float *> ptrToDelete;

  float *idepth[PYR_LEVELS];
  float *weight_sums[PYR_LEVELS];
  float *weight_sums_bak[PYR_LEVELS];

  Mat33f Ki[PYR_LEVELS];
  float fx[PYR_LEVELS];
  float fy[PYR_LEVELS];
  float cx[PYR_LEVELS];
  float cy[PYR_LEVELS];
  int w[PYR_LEVELS];
  int h[PYR_LEVELS];

  // pc buffers
  float *pc_u[PYR_LEVELS];
  float *pc_v[PYR_LEVELS];
  float *pc_idepth[PYR_LEVELS];
  float *pc_color[PYR_LEVELS];
  int pc_n[PYR_LEVELS];

  /**************************Scale Optimization***************************/
  void calcGSSSEScale(int lvl, float &H_out, float &b_out, float scale);
  Vec6 calcResScale(int lvl, float scale, float cutoffTH,
                    bool plot_img = false);

  // Transformation from frame0 to frame1
  SE3 tfmF0ToF1;

  // cam1 params
  float fx1[PYR_LEVELS];
  float fy1[PYR_LEVELS];
  float cx1[PYR_LEVELS];
  float cy1[PYR_LEVELS];

  // warped buffers
  float *scaleBufWarped_rx1;
  float *scaleBufWarped_rx2;
  float *scaleBufWarped_rx3;
  float *scaleBufWarped_dx;
  float *scaleBufWarped_dy;
  float *scaleBufWarped_residual;
  float *scaleBufWarped_weight;
  float *scaleBufWarped_ref_color;
  int scaleBufWarped_n;

  FrameHessian *fhStereo;
  ScaleAccumulator scaleAcc;
};

} // namespace dso
