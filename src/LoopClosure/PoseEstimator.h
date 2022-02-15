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

#include "OptimizationBackend/MatrixAccumulators.h"
#include "util/NumType.h"
#include "util/settings.h"

#include "LoopClosure/LoopHandler.h"

#include "vector"
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define INNER_PERCENT 90
// #define TRANS_THRES 3.0
// #define ROT_THRES 0.2

namespace dso {
struct FrameHessian;
struct PointFrameResidual;

struct LoopFrame;

class PoseEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  PoseEstimator(int w, int h);
  ~PoseEstimator();

  bool estimate(const LoopFrame *cur_frame, const LoopFrame *matched_frame,
                int coarsest_lvl, Eigen::Matrix4d &ref_to_new,
                float &pose_error);

  bool icp(const std::vector<Eigen::Vector3d> &pts_source,
           const std::vector<Eigen::Vector3d> &pts_target,
           Eigen::Matrix4d &tfm_target_source, float &icp_score);

private:
  void makeK(const std::vector<float> &cam);
  Vec6 calcRes(int lvl, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH,
               bool plot_img = false);
  void calcGSSSE(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew,
                 AffLight aff_g2l);

  float fx[PYR_LEVELS];
  float fy[PYR_LEVELS];
  float cx[PYR_LEVELS];
  float cy[PYR_LEVELS];
  int w[PYR_LEVELS];
  int h[PYR_LEVELS];

  // pc buffers
  std::vector<std::pair<Eigen::Vector3d, float *>> pts;

  // warped buffers
  float *bufWarped_idepth;
  float *bufWarped_u;
  float *bufWarped_v;
  float *bufWarped_dx;
  float *bufWarped_dy;
  float *bufWarped_residual;
  float *bufWarped_weight;
  float *bufWarped_ref_color;
  int bufWarped_n;

  Accumulator9 acc;

  std::vector<float *> ptrToDelete;

  // photometric terms
  AffLight refAffGToL;
  float refAbExposure;
  FrameHessian *newFrame;

  pcl::PointCloud<pcl::PointXYZ>
  transformPoints(const std::vector<Eigen::Vector3d> &pts_input,
                  const Eigen::Matrix4d &tfm_target_source);
};

} // namespace dso