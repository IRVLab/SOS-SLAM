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

#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <flann/flann.hpp>
#include <iostream>
#include <vector>

#include "LoopClosure/LoopHandler.h"
#include "util/settings.h"

namespace dso {

struct LoopFrame;

class ScanContext {
public:
  ScanContext();

  unsigned int getHeight();
  unsigned int getWidth();

  void process_scan(int cur_frame_id, const dso::SE3 &cur_wc,
                    std::vector<Eigen::Vector3d> &pts_spherical,
                    g2o::SE3Quat &tfm_sc_rig);

  bool generate(LoopFrame *frame, flann::Matrix<float> &ringkey);

  void search_ringkey(const flann::Matrix<float> &ringkey,
                      flann::Index<flann::L2<float>> *ringkeys,
                      std::vector<int> &candidates);

  void search_sc(std::vector<std::pair<int, double>> &signature,
                 const std::vector<LoopFrame *> &loop_frames,
                 const std::vector<int> &candidates, int &res_idx,
                 float &res_diff);

private:
  void getGravityByPCA(const std::vector<Eigen::Vector3d> &pts,
                       Mat33 &rot_ned_cam);

  void process_scan_forward(int cur_frame_id, const dso::SE3 &cur_wc,
                            std::vector<Eigen::Vector3d> &pts_spherical,
                            g2o::SE3Quat &tfm_sc_rig);

  void process_scan_downward(const dso::SE3 &cur_wc,
                             std::vector<Eigen::Vector3d> &pts_scan,
                             g2o::SE3Quat &tfm_sc_rig);

  // variables for forward-facing camera only
  std::unordered_map<int, Eigen::Matrix<double, 6, 1>> id2pose_wc;
  std::vector<std::pair<int, Eigen::Vector3d>> ptsNearby;
};

} // namespace dso
