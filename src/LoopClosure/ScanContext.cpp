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

#include "util/settings.h"

#include "ScanContext.h"
#include <cmath>
#include <limits>
#include <unordered_map>

// Scan Context resolution
#define NUM_S 60
#define NUM_R 20

// sphereical filtering
#define RES_X 1.0
#define RES_Y 0.5
#define RES_Z 1.0

#define CENTER_RANGE 2.0   // find the highest point within for alignment
#define VAR_HEIGHT_THRES 5 // minimal height variance for a meaningful signature

#define FLANN_NN 3
#define LOOP_MARGIN 100
#define RINGKEY_THRES 0.1

namespace dso {

ScanContext::ScanContext() {}

unsigned int ScanContext::getHeight() { return NUM_R; }
unsigned int ScanContext::getWidth() { return NUM_S; }

void ScanContext::process_scan(int cur_frame_id, const dso::SE3 &cur_wc,
                               std::vector<Eigen::Vector3d> &pts_sc,
                               g2o::SE3Quat &tfm_sc_rig) {
  if (setting_cam_mode == FORWARD_CAM) {
    process_scan_forward(cur_frame_id, cur_wc, pts_sc, tfm_sc_rig);
  } else {
    process_scan_downward(cur_wc, pts_sc, tfm_sc_rig);
  }
}

void ScanContext::getAlignTfmByPCA(const std::vector<Eigen::Vector3d> &pts,
                                   Mat44 &tfm_ned_cam) {
  Vec3 center;
  for (auto &pc : pts) {
    center += pc;
  }
  center /= pts.size();

  // normalize pts (zero mean)
  Eigen::MatrixXd pts_mat(pts.size(), 3);
  for (size_t i = 0; i < pts.size(); i++) {
    pts_mat.row(i) = pts[i] - center;
  }

  // PCA
  auto cov = pts_mat.transpose() * pts_mat;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);

  // compute the rotation matrix
  Vec3 rot_cam_nedx, rot_cam_nedy, rot_cam_nedz;
  rot_cam_nedz = tfm_ned_cam.block<1, 3>(2, 0);
  if (!setting_enable_imu) {
    rot_cam_nedz = es.eigenvectors().col(0);
    if (rot_cam_nedz.sum() < 0) {
      rot_cam_nedz = -rot_cam_nedz;
    }
  }

  std::vector<Vec3> rot_cam_nedy_candi = {
      es.eigenvectors().col(1), -es.eigenvectors().col(1),
      es.eigenvectors().col(2), -es.eigenvectors().col(2)};
  rot_cam_nedy = rot_cam_nedy_candi[0];
  for (int i = 1; i < 4; i++) {
    if (rot_cam_nedy(0) < rot_cam_nedy_candi[i](0)) { // dot prod [1 0 0]
      rot_cam_nedy = rot_cam_nedy_candi[i];
    }
  }
  rot_cam_nedy = rot_cam_nedy - rot_cam_nedz.dot(rot_cam_nedy) * rot_cam_nedz;
  rot_cam_nedy.normalize();

  rot_cam_nedx = rot_cam_nedy.cross(rot_cam_nedz);

  tfm_ned_cam.setIdentity();
  tfm_ned_cam.block<1, 3>(0, 0) = rot_cam_nedx;
  tfm_ned_cam.block<1, 3>(1, 0) = rot_cam_nedy;
  tfm_ned_cam.block<1, 3>(2, 0) = rot_cam_nedz;
  tfm_ned_cam.topRightCorner<3, 1>() =
      -tfm_ned_cam.topLeftCorner<3, 3>() * center;
}

void ScanContext::process_scan_forward(
    int cur_frame_id, const dso::SE3 &cur_wc,
    std::vector<Eigen::Vector3d> &pts_spherical, g2o::SE3Quat &tfm_sc_rig) {
  id2pose_wc[cur_frame_id] = cur_wc.log();

  // cache new pts_spherical into ptsNearby
  for (size_t i = 0; i < pts_spherical.size(); i++) {
    Eigen::Vector3d p_g =
        cur_wc.rotationMatrix() * pts_spherical[i] + cur_wc.translation();
    ptsNearby.emplace_back(std::pair<int, Eigen::Vector3d>(cur_frame_id, p_g));
  }
  pts_spherical.clear();

  // if the oriention difference between a historical keyframe and current
  // keyframe is large, trim all associated points
  for (auto &ip : id2pose_wc) {
    auto pose_diff_se3 = (cur_wc.inverse() * dso::SE3::exp(ip.second)).log();
    auto rotation_norm = pose_diff_se3.tail(3).norm();
    if (rotation_norm > 0.5) {
      id2pose_wc.erase(ip.first);
    }
  }

  // get/filter spherical points
  std::vector<double> steps{1.0 / RES_X, 1.0 / RES_Y, 1.0 / RES_Z};
  std::vector<int> voxel_size{
      static_cast<int>(floor(2 * setting_lidar_range * steps[0]) + 1),
      static_cast<int>(floor(2 * setting_lidar_range * steps[1]) + 1),
      static_cast<int>(floor(2 * setting_lidar_range * steps[2]) + 1)};
  std::vector<int> loc_step{1, voxel_size[0], voxel_size[0] * voxel_size[1]};
  std::unordered_map<int, std::pair<int, Eigen::Vector3d>> loc2idx_pt;
  for (size_t i = 0; i < ptsNearby.size(); i++) {
    if (id2pose_wc.find(ptsNearby[i].first) == id2pose_wc.end()) {
      continue; // orientation changed too much
    }

    Eigen::Vector4d p_g(ptsNearby[i].second(0), ptsNearby[i].second(1),
                        ptsNearby[i].second(2), 1.0);
    Eigen::Vector3d p_l = cur_wc.inverse().matrix3x4() * p_g;

    if (p_l.norm() >= setting_lidar_range) {
      continue; // out of range
    }

    // voxel indices
    int xi = static_cast<int>(floor((p_l(0) + setting_lidar_range) * steps[0]));
    int yi = static_cast<int>(floor((p_l(1) + setting_lidar_range) * steps[1]));
    int zi = static_cast<int>(floor((p_l(2) + setting_lidar_range) * steps[2]));
    int loc = xi * loc_step[0] + yi * loc_step[1] + zi * loc_step[2];

    // store the highest points
    if (loc2idx_pt.find(loc) == loc2idx_pt.end() ||
        -loc2idx_pt[loc].second(1) < -p_l(1)) {
      loc2idx_pt[loc] = {i, p_l};
    }
  }

  // output useful points
  std::vector<std::pair<int, Eigen::Vector3d>> new_pts_nearby;
  for (auto &l_ip : loc2idx_pt) {
    pts_spherical.push_back(l_ip.second.second);
    new_pts_nearby.push_back(ptsNearby[l_ip.second.first]);
  }

  // mark tfm_sc_rig
  Mat44 tfm_ned_cam = cur_wc.matrix();
  getAlignTfmByPCA(pts_spherical, tfm_ned_cam);
  tfm_sc_rig = g2o::SE3Quat(tfm_ned_cam.topLeftCorner<3, 3>(),
                            tfm_ned_cam.topRightCorner<3, 1>());

  // update nearby pts
  ptsNearby = new_pts_nearby;
}

void ScanContext::process_scan_downward(const dso::SE3 &cur_wc,
                                        std::vector<Eigen::Vector3d> &pts_sc,
                                        g2o::SE3Quat &tfm_sc_rig) {
  Mat44 tfm_ned_cam = cur_wc.matrix();
  getAlignTfmByPCA(pts_sc, tfm_ned_cam);
  Mat33 rot_ned_cam = tfm_ned_cam.topLeftCorner<3, 3>();

  // rotate points into NED frame
  for (auto &pt : pts_sc) {
    pt = rot_ned_cam * pt;
  }

  // find the center of the scan
  Eigen::Vector2d center;
  center.setZero();
  for (auto &pt : pts_sc) {
    center += pt.head(2);
  }
  center /= pts_sc.size();
  // std::cout << center.transpose() << std::endl;

  // find the alignment point on the horizontal plane
  Eigen::Vector3d align_pt;
  align_pt(2) = 1e5; // NED coordinate, highest z = min_z
  for (auto &pt : pts_sc) {
    if (((pt.head(2) - center).norm() < CENTER_RANGE) &&
        (align_pt(2) > pt(2))) {
      align_pt = pt;
    }
  }
  for (auto &pt : pts_sc) {
    pt.head(2) -= align_pt.head(2);
  }

  // trim points
  std::vector<Eigen::Vector3d> pts_scan_trimmed;
  double sum_z = 0.0;
  for (auto &pt : pts_sc) {
    if (pt.head(2).norm() < setting_lidar_range) {
      pts_scan_trimmed.push_back(pt);
      sum_z += pt(2);
    }
  }
  pts_sc = pts_scan_trimmed;
  align_pt(2) = sum_z / pts_sc.size();
  // std::cout << align_pt.transpose() << std::endl << std::endl;

  // normalize height
  for (auto &pt : pts_sc) {
    pt(2) -= align_pt(2);
  }

  tfm_sc_rig = g2o::SE3Quat(rot_ned_cam, -align_pt);

  // transform points back to camera frame
  for (auto &pt : pts_sc) {
    pt = tfm_sc_rig.inverse() * pt;
  }
}

bool ScanContext::generate(LoopFrame *frame, flann::Matrix<float> &ringkey) {
  // ringkey
  ringkey = flann::Matrix<float>(new float[NUM_R], 1, NUM_R);
  for (int i = 0; i < NUM_R; i++) {
    ringkey[0][i] = 0.0;
  }

  // signature matrix
  MatXX sig_height = DBL_MAX * MatXX::Ones(NUM_S, NUM_R);
  for (const auto &pt_cam : frame->pts_sc) // loop on pts
  {
    // projection from camera (rig) coordinate to NED coordinate
    Eigen::Vector3d pt_ned = frame->tfm_sc_rig * pt_cam;
    double i_n = pt_ned(0);
    double i_e = pt_ned(1);
    double i_d = pt_ned(2);

    double theta = std::atan2(i_e, i_n);
    while (theta < 0)
      theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI)
      theta -= 2.0 * M_PI;
    int si = theta / (2.0 * M_PI) * NUM_S;
    int ri = std::sqrt(i_n * i_n + i_e * i_e) / setting_lidar_range * NUM_R;
    if (ri >= NUM_R) // happens because PCA translated the points
      continue;

    assert(si < NUM_S);
    assert(ri < NUM_R);
    sig_height(si, ri) = std::min(sig_height(si, ri), i_d);
  }

  Eigen::VectorXd sig_norm_si = Eigen::VectorXd::Zero(NUM_S);
  double ave_height = 0.0;
  for (int i = 0; i < NUM_S; i++) {
    for (int j = 0; j < NUM_R; j++) {
      if (sig_height(i, j) < DBL_MAX) {
        ringkey[0][j]++; // calculate ringkey
        sig_norm_si(i) += sig_height(i, j) * sig_height(i, j);
        ave_height += sig_height(i, j);
      }
    }
  }
  sig_norm_si = sig_norm_si.cwiseSqrt();
  ave_height /= frame->signature.size();

  // normalize ringkey
  for (int i = 0; i < NUM_R; i++) {
    ringkey[0][i] /= NUM_S;
  }

  // get the aligned and normalized signature
  double var_height = 0.0;
  for (int i = 0; i < NUM_S; i++) {
    for (int j = 0; j < NUM_R; j++) {
      if (sig_height(i, j) < DBL_MAX) {
        frame->signature.push_back(
            {i * NUM_R + j, sig_height(i, j) / sig_norm_si(i)});

        double dh = sig_height(i, j) - ave_height;
        var_height += dh * dh;
      }
    }
  }
  var_height /= frame->signature.size();

  // std::cout << std::endl << "var_height " << var_height << std::endl;
  return var_height > VAR_HEIGHT_THRES;
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
                            const std::vector<LoopFrame *> &frames,
                            const std::vector<int> &candidates, int &res_idx,
                            float &res_diff) {
  res_idx = candidates[0];
  res_diff = 1.1;
  for (auto &candidate : candidates) {
    // Compute difference by Euclidean distance
    float cur_prod = 0;
    size_t m(0), n(0);
    std::vector<std::pair<int, double>> &candi_sig =
        frames[candidate]->signature;
    // siganture is a sparse vector <index, value>
    while (m < signature.size() && n < candi_sig.size()) {
      if (signature[m].first == candi_sig[n].first) {
        cur_prod += signature[m++].second * candi_sig[n++].second;
      } else {
        signature[m].first < candi_sig[n].first ? m++ : n++;
      }
    }

    float cur_diff = (1 - cur_prod / NUM_S) / 2.0;
    if (res_diff > cur_diff) {
      res_idx = candidate;
      res_diff = cur_diff;
    }
  }
}

} // namespace dso