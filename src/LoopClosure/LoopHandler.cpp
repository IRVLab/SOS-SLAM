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

#include "LoopHandler.h"

#include <fstream>
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

namespace dso {
LoopHandler::LoopHandler(IOWrap::PangolinSOSVIOViewer *pangolin_viewer)
    : pangolinViewer(pangolin_viewer) {
  // place recognition
  scPtr = new ScanContext();
  flann::Matrix<float> init_data(new float[scPtr->getHeight()], 1,
                                 scPtr->getHeight());
  ringkeys = new flann::Index<flann::L2<float>>(init_data,
                                                flann::KDTreeIndexParams(4));
  ringkeys->buildIndex();

  poseEstimator = new PoseEstimator(wG[0], hG[0]);

  // setup poseOptimizer
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver;
  linearSolver = g2o::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  g2o::OptimizationAlgorithmLevenberg *algorithm =
      new g2o::OptimizationAlgorithmLevenberg(
          g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
  poseOptimizer.setAlgorithm(algorithm);
  poseOptimizer.setVerbose(false);

  running = true;
  runThread = boost::thread(&LoopHandler::run, this);

  directLoopCount = 0;
  icpLoopCount = 0;
}

void LoopHandler::savePose() {
  running = false;

  // Export the final pose graph
  std::ofstream sodso_file, dslam_file;
  sodso_file.open("sodso.txt");
  dslam_file.open("dslam.txt");
  sodso_file << std::setprecision(6);
  dslam_file << std::setprecision(6);
  for (auto &lf : loopFrames) {
    auto t_wc = lf->trans_w_c_orig;
    sodso_file << lf->incoming_id << " ";
    sodso_file << t_wc(0) << " " << t_wc(1) << " " << t_wc(2) << std::endl;

    t_wc = lf->tfm_w_c.translation();
    dslam_file << lf->incoming_id << " ";
    dslam_file << t_wc(0) << " " << t_wc(1) << " " << t_wc(2) << std::endl;
  }
  sodso_file.close();
  dslam_file.close();
}

LoopHandler::~LoopHandler() {
  running = false;
  runThread.join();

  delete ringkeys;
  delete scPtr;

  delete poseEstimator;

  for (LoopFrame *lf : loopFrames) {
    delete lf;
  }
}

void LoopHandler::join() {
  runThread.join();
  printf("JOINED LoopHandler thread!\n");
}

void LoopHandler::optimize() {
  if (loopFrames.empty()) {
    return;
  }

  // poseOptimizer.clear();
  // Vertices
  for (LoopFrame *lf : loopFrames) {
    if (lf->graph_added) {
      continue;
    }

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(lf->kf_id);
    v->setEstimate(lf->tfm_w_c);
    poseOptimizer.addVertex(v);
    lf->graph_added = true;

    // no constraint for the first node of each new sequence
    if (isnan(lf->fh->dso_error)) {
      continue;
    }

    // Edges
    for (LoopEdge *le : lf->edges) {
      g2o::EdgeSE3 *e = new g2o::EdgeSE3();
      e->setVertex(0, v);
      e->setVertex(1, poseOptimizer.vertex(le->id_from));
      e->setMeasurement(le->measurement);
      e->setInformation(le->information);
      e->setRobustKernel(new g2o::RobustKernelHuber());
      poseOptimizer.addEdge(e);
    }
  }

  // fix last vertex
  poseOptimizer.vertex(loopFrames.back()->kf_id)->setFixed(true);

  poseOptimizer.initializeOptimization();
  poseOptimizer.computeInitialGuess();
  poseOptimizer.optimize(25);
}

void LoopHandler::publishKeyframes(FrameHessian *fh, CalibHessian *HCalib) {
  float fx = HCalib->fxl();
  float fy = HCalib->fyl();
  float cx = HCalib->cxl();
  float cy = HCalib->cyl();

  Mat33 rot_w_c = fh->shell->camToWorld.rotationMatrix();

  // points for loop closure
  std::vector<std::pair<Eigen::Vector3d, float *>> pts_dso;
  std::vector<Eigen::Vector3d> pts_sc;
  Eigen::Vector3d align_pt;

  if (setting_lidar_range > 0 && fh->scale_error > 0) {
    pts_sc = fh->points;
    for (PointHessian *p : fh->pointHessiansMarginalized) {
      pts_sc.push_back({p->u, p->v, p->idepth_scaled});
    }

    for (size_t i = 0; i < pts_sc.size(); i++) {
      // point in camera frame
      Eigen::Vector3d p_c((pts_sc[i](0) - cx) / fx / pts_sc[i](2),
                          (pts_sc[i](1) - cy) / fy / pts_sc[i](2),
                          1 / pts_sc[i](2));

      float *dIp = new float[PYR_LEVELS];
      for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        float u = (pts_sc[i](0) + 0.5) / ((int)1 << lvl) - 0.5;
        float v = (pts_sc[i](1) + 0.5) / ((int)1 << lvl) - 0.5;
        dIp[lvl] = getInterpolatedElement31(fh->dIp[lvl], u, v, wG[lvl]);
      }
      pts_dso.emplace_back(std::pair<Eigen::Vector3d, float *>(p_c, dIp));

      // align to NED frame
      pts_sc[i] = rot_w_c * p_c;
    }

    auto t0 = std::chrono::steady_clock::now();
    scPtr->process_scan(pts_sc, align_pt);
    auto t1 = std::chrono::steady_clock::now();
    ptsGenerationTime.emplace_back(t1 - t0);
  }
  fh->points.clear();

  LoopFrame *cur_frame =
      new LoopFrame(fh, {fx, fy, cx, cy}, pts_dso, pts_sc, align_pt);
  boost::unique_lock<boost::mutex> lk_lfq(loopFrameQueueMutex);
  loopFrameQueue.emplace(cur_frame);
}

void LoopHandler::run() {
  std::cout << "Start Loop Thread" << std::endl;
  while (running) {
    boost::unique_lock<boost::mutex> lk_lfq(loopFrameQueueMutex);
    if (loopFrameQueue.empty()) {
      lk_lfq.unlock();
      usleep(5000);
      continue;
    }
    LoopFrame *cur_frame = loopFrameQueue.front();
    loopFrameQueue.pop();

    // for Pangolin visualization
    std::vector<Eigen::Vector3d> lidar_pts = cur_frame->pts_sc;

    loopFrames.emplace_back(cur_frame);
    // Connection to previous keyframe
    if (loopFrames.size() > 1) {
      auto prv_frame = loopFrames[loopFrames.size() - 2];
      g2o::SE3Quat tfm_prv_cur =
          (prv_frame->tfm_w_c.inverse() * cur_frame->tfm_w_c);
      cur_frame->edges.emplace_back(
          new LoopEdge(prv_frame->kf_id, tfm_prv_cur.inverse(),
                       DSO_ERROR_SCALE * cur_frame->fh->dso_error,
                       SCALE_ERROR_SCALE * cur_frame->fh->scale_error));
    }

    // Loop closure is disabled if lidar_range < 0 or scale optimization failed
    if (setting_lidar_range < 0 || cur_frame->fh->scale_error < 0) {
      delete cur_frame->fh;
      usleep(5000);
      continue;
    }

    /* == Get ringkey and signature from the aligned points by Scan Context = */
    flann::Matrix<float> ringkey;
    std::vector<std::pair<int, double>> signature;
    auto t0 = std::chrono::steady_clock::now();
    scPtr->generate(cur_frame->pts_sc, ringkey, signature);
    auto t1 = std::chrono::steady_clock::now();
    scGenerationTime.emplace_back(t1 - t0);
    cur_frame->signature = signature;

    /* ======================== Loop Closure ================================ */
    // fast search by ringkey
    std::vector<int> ringkey_candidates;
    t0 = std::chrono::steady_clock::now();
    scPtr->search_ringkey(ringkey, ringkeys, ringkey_candidates);
    t1 = std::chrono::steady_clock::now();
    searchRingkeyTime.emplace_back(t1 - t0);

    if (!ringkey_candidates.empty()) {
      // search by ScanContext
      t0 = std::chrono::steady_clock::now();
      int matched_idx;
      float sc_diff;
      scPtr->search_sc(signature, loopFrames, ringkey_candidates,
                       scPtr->getWidth(), matched_idx, sc_diff);
      t1 = std::chrono::steady_clock::now();
      searchScTime.emplace_back(t1 - t0);

      if (sc_diff < setting_scan_context_thres) {
        auto matched_frame = loopFrames[matched_idx];
        printf("%4d - %4d  SC: %.3f  ", cur_frame->incoming_id,
               matched_frame->incoming_id, sc_diff);

        // calculate the initial tfm_matched_cur from ScanContext
        Eigen::Matrix4d tfm_cur_matched =
            (cur_frame->tfm_scan_rig.inverse() * matched_frame->tfm_scan_rig)
                .to_homogeneous_matrix();

        // first try direct alignment
        Eigen::Matrix4d tfm_cur_matched_direct = tfm_cur_matched;
        float pose_error;
        t0 = std::chrono::steady_clock::now();
        bool direct_succ = poseEstimator->estimate(
            matched_frame->pts_dso, matched_frame->ab_exposure, cur_frame->fh,
            cur_frame->cam, pyrLevelsUsed - 1, tfm_cur_matched_direct,
            pose_error);
        t1 = std::chrono::steady_clock::now();
        directEstTime.emplace_back(t1 - t0);

        // try icp if direct alignment failed
        bool icp_succ = false;
        Eigen::Matrix4d tfm_cur_matched_icp = tfm_cur_matched;
        if (!direct_succ) {
          t0 = std::chrono::steady_clock::now();
          icp_succ =
              poseEstimator->icp(matched_frame->pts_sc, cur_frame->pts_sc,
                                 tfm_cur_matched_icp, pose_error);
          t1 = std::chrono::steady_clock::now();
          icpTime.emplace_back(t1 - t0);
        }

        if (direct_succ || icp_succ) {
          if (direct_succ) {
            directLoopCount++;
            tfm_cur_matched = tfm_cur_matched_direct;
            pose_error *= DIRECT_ERROR_SCALE;
            printf("            add loop\n");
          } else {
            icpLoopCount++;
            tfm_cur_matched = tfm_cur_matched_icp;
            pose_error *= ICP_ERROR_SCALE;
            printf("add loop\n");
          }

          // add the loop constraint
          cur_frame->edges.emplace_back(new LoopEdge(
              matched_frame->kf_id,
              g2o::SE3Quat(tfm_cur_matched.block<3, 3>(0, 0),
                           tfm_cur_matched.block<3, 1>(0, 3)),
              pose_error, SCALE_ERROR_SCALE * matched_frame->fh->scale_error));

          // run pose graph optimization
          t0 = std::chrono::steady_clock::now();
          optimize();
          t1 = std::chrono::steady_clock::now();
          optTime.emplace_back(t1 - t0);

          // update the trajectory
          for (auto &lf : loopFrames) {
            g2o::VertexSE3 *v =
                (g2o::VertexSE3 *)poseOptimizer.vertex(lf->kf_id);
            auto new_tfm_wc = v->estimate().matrix();
            lf->tfm_w_c = g2o::SE3Quat(new_tfm_wc.block<3, 3>(0, 0),
                                       new_tfm_wc.block<3, 1>(0, 3));
            if (pangolinViewer) {
              pangolinViewer->modifyKeyframePoseByKFID(lf->kf_id,
                                                       SE3(new_tfm_wc));
            }
          }

          // merge matech lidar pts for Pangolin visualization
          Eigen::Matrix<double, 4, 1> pt_matched, pt_cur;
          pt_matched(3) = 1.0;
          for (size_t i = 0; i < matched_frame->pts_sc.size(); i++) {
            pt_matched.head(3) = matched_frame->pts_sc[i];
            pt_cur = tfm_cur_matched * pt_matched;
            lidar_pts.push_back({pt_cur(0), pt_cur(1), pt_cur(2)});
          }
        } else {
          printf("\n");
        }
      }
    }
    if (pangolinViewer) {
      pangolinViewer->refreshLidarData(lidar_pts, cur_frame->pts_sc.size());
    }
    delete cur_frame->fh;
    usleep(5000);
  }
  std::cout << "Finished Loop Thread" << std::endl;
}

} // namespace dso