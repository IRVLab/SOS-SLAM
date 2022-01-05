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

#define CENTER_RANGE 2.0 // find the highest point within for alignment
#define FLANN_NN 3
#define LOOP_MARGIN 100
#define RINGKEY_THRES 0.1

namespace dso {

struct LoopFrame;

class ScanContext {
public:
  ScanContext();
  ScanContext(int s, int r);

  unsigned int getHeight();
  unsigned int getWidth();

  void process_scan(std::vector<Eigen::Vector3d> &pts_scan,
                    Eigen::Vector3d &align_pt);

  void generate(const std::vector<Eigen::Vector3d> &pts_spherical,
                flann::Matrix<float> &ringkey, std::vector<std::pair<int, double>> &signature);

  void search_ringkey(const flann::Matrix<float> &ringkey,
                      flann::Index<flann::L2<float>> *ringkeys,
                      std::vector<int> &candidates);

  void search_sc(std::vector<std::pair<int, double>> &signature,
                 const std::vector<LoopFrame *> &loop_frames,
                 const std::vector<int> &candidates, int sc_width, int &res_idx,
                 float &res_diff);

private:
  int numS;
  int numR;
};

} // namespace dso
