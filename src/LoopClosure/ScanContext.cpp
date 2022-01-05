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

#include "util/settings.h"

#include "ScanContext.h"
#include <cmath>
#include <limits>
#include <unordered_map>

namespace dso {

ScanContext::ScanContext() {
  numS = 60;
  numR = 20;
}

ScanContext::ScanContext(int s, int r) : numS(s), numR(r) {}

unsigned int ScanContext::getHeight() { return numR; }
unsigned int ScanContext::getWidth() { return numS; }

void ScanContext::process_scan(std::vector<Eigen::Vector3d> &pts_scan,
                               Eigen::Vector3d &align_pt) {
  // find the center of the scan
  Eigen::Vector2d center;
  center.setZero();
  for (auto &pt : pts_scan) {
    center += pt.head(2);
  }
  center /= pts_scan.size();
  // std::cout << center.transpose() << std::endl;

  // find the alignment point
  align_pt(2) = 1e5; // NED coordinate, highest z = min_z
  for (auto &pt : pts_scan) {
    if (((pt.head(2) - center).norm() < CENTER_RANGE) &&
        (align_pt(2) > pt(2))) {
      align_pt = pt;
    }
  }
  // std::cout << align_pt.transpose() << std::endl << std::endl;
  for (auto &pt : pts_scan) {
    pt -= align_pt;
  }

  // trim points
  std::vector<Eigen::Vector3d> pts_scan_trimmed;
  for (auto &pt : pts_scan) {
    if (pt.head(2).norm() < setting_lidar_range) {
      pts_scan_trimmed.push_back(pt);
    }
  }
  pts_scan = pts_scan_trimmed;
}

void ScanContext::generate(const std::vector<Eigen::Vector3d> &pts_scan,
                           flann::Matrix<float> &ringkey, std::vector<std::pair<int, double>> &signature) {

  // ringkey
  ringkey = flann::Matrix<float>(new float[numR], 1, numR);
  for (int i = 0; i < numR; i++) {
    ringkey[0][i] = 0.0;
  }

  // signature matrix
  Eigen::VectorXd sig_height = std::numeric_limits<double>::max() *
                               Eigen::VectorXd::Ones(numS * numR);

  for (size_t i = 0; i < pts_scan.size(); i++) // loop on pts
  {
    // projection to polar coordinate
    double yp = pts_scan[i](0);
    double zp = pts_scan[i](1);

    double theta = std::atan2(zp, yp);
    while (theta < 0)
      theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI)
      theta -= 2.0 * M_PI;
    int si = theta / (2.0 * M_PI) * numS;
    int ri = std::sqrt(yp * yp + zp * zp) / setting_lidar_range * numR;
    assert(si < numS);
    assert(ri < numR);

    sig_height(si * numR + ri) =
        std::min(sig_height(si * numR + ri), pts_scan[i](2));
  }

  // calculate ringkey and signature
  Eigen::VectorXd sig_norm_si = Eigen::VectorXd::Zero(numS);
  for (int i = 0; i < numS * numR; i++) {
    if (sig_height(i) < std::numeric_limits<double>::max()) {
      ringkey[0][i % numR]++;
      signature.push_back({i, sig_height(i)});
      sig_norm_si(i / numR) += sig_height(i) * sig_height(i);
    }
  }

  // normalize ringkey
  for (int i = 0; i < numR; i++) {
    ringkey[0][i] /= numS;
  }

  // normalize signature
  sig_norm_si = sig_norm_si.cwiseSqrt();
  for (size_t i = 0; i < signature.size(); i++) {
    assert(sig_norm_si(signature[i].first / numR) > 0);
    signature[i].second /= sig_norm_si(signature[i].first / numR);
  }
}

void ScanContext::search_ringkey(const flann::Matrix<float> &ringkey,
                                 flann::Index<flann::L2<float>> *ringkeys,
                                 std::vector<int> &candidates) {
  // query ringkey
  if (ringkeys->size() > FLANN_NN) {
    flann::Matrix<int> idces(new int[FLANN_NN], 1, FLANN_NN);
    flann::Matrix<float> dists(new float[FLANN_NN], 1, FLANN_NN);
    ringkeys->knnSearch(ringkey, idces, dists, FLANN_NN,
                        flann::SearchParams(128));
    for (int i = 0; i < FLANN_NN; i++) {
      if (dists[0][i] < RINGKEY_THRES && idces[0][i] > 0) {
        candidates.emplace_back(idces[0][i] - 1);
      }
    }
  }

  // store ringkey in waiting queue of size LOOP_MARGIN
  int r_cols = ringkey.cols;
  static int queue_idx = 0;
  static flann::Matrix<float> ringkey_queue(new float[LOOP_MARGIN * r_cols],
                                            LOOP_MARGIN, r_cols);
  if (queue_idx >= LOOP_MARGIN) {
    flann::Matrix<float> ringkey_to_add(new float[r_cols], 1, r_cols);
    for (int j = 0; j < r_cols; j++) {
      ringkey_to_add[0][j] = ringkey_queue[queue_idx % LOOP_MARGIN][j];
    }
    ringkeys->addPoints(ringkey_to_add);
  }
  for (int j = 0; j < r_cols; j++) {
    ringkey_queue[queue_idx % LOOP_MARGIN][j] = ringkey[0][j];
  }
  queue_idx++;
}

void ScanContext::search_sc(std::vector<std::pair<int, double>> &signature,
                            const std::vector<LoopFrame *> &loop_frames,
                            const std::vector<int> &candidates, int sc_width,
                            int &res_idx, float &res_diff) {
  res_idx = candidates[0];
  res_diff = 1.1;
  for (auto &candidate : candidates) {
    // Compute difference by Euclidean distance
    float cur_prod = 0;
    size_t m(0), n(0);
    std::vector<std::pair<int, double>> &candi_sig = loop_frames[candidate]->signature;
    // siganture is a sparse vector <index, value>
    while (m < signature.size() && n < candi_sig.size()) {
      if (signature[m].first == candi_sig[n].first) {
        cur_prod += signature[m++].second * candi_sig[n++].second;
      } else {
        signature[m].first < candi_sig[n].first ? m++ : n++;
      }
    }

    float cur_diff = (1 - cur_prod / sc_width) / 2.0;
    if (res_diff > cur_diff) {
      res_idx = candidate;
      res_diff = cur_diff;
    }
  }
}

} // namespace dso