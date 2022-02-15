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

#include "LoopHandler.h"

#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <fstream>

namespace dso {
namespace IOWrap {

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

  currentPosePublisher = rosNode.advertise<geometry_msgs::PoseStamped>(
      "pose_cam0_in_world/current", 1000);
  marginalizedPosePublisher = rosNode.advertise<geometry_msgs::PoseStamped>(
      "pose_cam0_in_world/marginalized", 1000);

  curFrameID = -1;
}

void LoopHandler::savePose() {
  printf("Writing poses...\n");
  running = false;

  // Export the final pose graph
  std::ofstream pose_file;
  pose_file.open("poses.txt");
  pose_file << std::setprecision(6);
  for (auto &lf : loopFrames) {
    auto t_wc = lf->tfm_w_c.translation();
    pose_file << lf->incoming_id << " ";
    pose_file << t_wc(0) << " " << t_wc(1) << " " << t_wc(2) << std::endl;
  }
  pose_file.close();
}

LoopHandler::~LoopHandler() {
  savePose();

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

void LoopHandler::publishKeyframes(std::vector<FrameHessian *> &frames,
                                   bool final, CalibHessian *HCalib) {
  FrameHessian *fh = frames.back();

  /******************************* Publish Pose *******************************/
  SE3 camToWorld = fh->shell->camToWorldScaled;

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.seq = fh->shell->incoming_id;
  pose_msg.header.stamp = ros::Time(fh->shell->timestamp);
  pose_msg.header.frame_id = "camera0";
  pose_msg.pose.position.x = camToWorld.translation()(0);
  pose_msg.pose.position.y = camToWorld.translation()(1);
  pose_msg.pose.position.z = camToWorld.translation()(2);
  pose_msg.pose.orientation.x = camToWorld.unit_quaternion().x();
  pose_msg.pose.orientation.y = camToWorld.unit_quaternion().y();
  pose_msg.pose.orientation.z = camToWorld.unit_quaternion().z();
  pose_msg.pose.orientation.w = camToWorld.unit_quaternion().w();
  if (!final) {
    currentPosePublisher.publish(pose_msg);
    return;
  }
  marginalizedPosePublisher.publish(pose_msg);

  if (setting_cam_mode == FORWARD_CAM) {
    if (curFrameID >= fh->frameID) {
      return;
    }
    curFrameID = fh->frameID;
  }

  /******************************* Loop Closure *******************************/
  assert(frames.size() == 1);
  float fx = HCalib->fxl();
  float fy = HCalib->fyl();
  float cx = HCalib->cxl();
  float cy = HCalib->cyl();

  // loop closure
  g2o::SE3Quat tfm_sc_rig;
  std::vector<Eigen::Vector3d> pts_sc;
  std::vector<std::pair<Eigen::Vector3d, float *>> pts_dso;
  if (setting_enable_loop_closure && fh->scale_error > 0) {
    /* ====================== Extract points ================================ */
    // points in [u,v,d] frame
    std::vector<Eigen::Vector3d> pts_uvd;
    if (setting_cam_mode == DOWNWARD_CAM) {
      pts_uvd = fh->points;
    }
    for (PointHessian *p : fh->pointHessiansMarginalized) {
      pts_uvd.push_back({p->u, p->v, p->idepth_scaled});
    }

    // points in camera frame
    for (size_t i = 0; i < pts_uvd.size(); i++) {
      Eigen::Vector3d p_c((pts_uvd[i](0) - cx) / fx / pts_uvd[i](2),
                          (pts_uvd[i](1) - cy) / fy / pts_uvd[i](2),
                          1 / pts_uvd[i](2));
      pts_sc.emplace_back(p_c);

      float *dIp = new float[PYR_LEVELS];
      for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
        float u = (pts_uvd[i](0) + 0.5) / ((int)1 << lvl) - 0.5;
        float v = (pts_uvd[i](1) + 0.5) / ((int)1 << lvl) - 0.5;
        dIp[lvl] = getInterpolatedElement31(fh->dIp[lvl], u, v, wG[lvl]);
      }
      pts_dso.emplace_back(std::pair<Eigen::Vector3d, float *>(p_c, dIp));
    }

    // Preprocess points
    scPtr->process_scan(curFrameID, camToWorld, pts_sc, tfm_sc_rig);
  }
  fh->points.clear();

  LoopFrame *cur_frame =
      new LoopFrame(fh, {fx, fy, cx, cy}, tfm_sc_rig, pts_dso, pts_sc);
  boost::unique_lock<boost::mutex> lk_lfq(loopFrameQueueMutex);
  loopFrameQueue.emplace(cur_frame);
}

void LoopHandler::run() {
  while (running) {
    boost::unique_lock<boost::mutex> lk_lfq(loopFrameQueueMutex);
    if (loopFrameQueue.empty()) {
      lk_lfq.unlock();
      usleep(5000);
      continue;
    }
    LoopFrame *cur_frame = loopFrameQueue.front();
    loopFrameQueue.pop();

    loopFrames.emplace_back(cur_frame);
    if (setting_enable_loop_closure) {
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

      // Loop closure is disabled if lidar_range < 0 or scale optimization
      // failed
      if (setting_lidar_range < 0 || cur_frame->fh->scale_error < 0) {
        delete cur_frame->fh;
        usleep(5000);
        continue;
      }

      // for Pangolin visualization
      std::vector<Eigen::Vector3d> lidar_pts = cur_frame->pts_sc;

      /* ======================== Scan Context ================================
       */
      flann::Matrix<float> ringkey;
      auto t0 = std::chrono::steady_clock::now();
      if (scPtr->generate(cur_frame, ringkey)) {
        auto t1 = std::chrono::steady_clock::now();
        scGenerationTime.emplace_back(t1 - t0);

        /* ======================= Loop Closure ===============================
         */
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
          scPtr->search_sc(cur_frame->signature, loopFrames, ringkey_candidates,
                           matched_idx, sc_diff);
          t1 = std::chrono::steady_clock::now();
          searchScTime.emplace_back(t1 - t0);

          if (sc_diff < setting_scan_context_thres) {
            auto matched_frame = loopFrames[matched_idx];
            printf("%d - %d: SC: %.2f  ", cur_frame->incoming_id,
                   matched_frame->incoming_id, sc_diff);

            // calculate the initial tfm_matched_cur from ScanContext
            Eigen::Matrix4d tfm_cur_matched =
                (cur_frame->tfm_sc_rig.inverse() * matched_frame->tfm_sc_rig)
                    .to_homogeneous_matrix();

            // first try direct alignment
            Eigen::Matrix4d tfm_cur_matched_direct = tfm_cur_matched;
            float pose_error;
            t0 = std::chrono::steady_clock::now();
            bool direct_succ = poseEstimator->estimate(
                cur_frame, matched_frame, pyrLevelsUsed - 1,
                tfm_cur_matched_direct, pose_error);
            t1 = std::chrono::steady_clock::now();
            directEstTime.emplace_back(t1 - t0);

            // try icp if direct alignment failed
            bool icp_succ = false;
            Eigen::Matrix4d tfm_cur_matched_icp =
                direct_succ ? tfm_cur_matched_direct : tfm_cur_matched;
            direct_succ = direct_succ && !setting_loop_force_icp;
            if (!direct_succ) {
              t0 = std::chrono::steady_clock::now();
              icp_succ =
                  poseEstimator->icp(matched_frame->pts_sc, cur_frame->pts_sc,
                                     tfm_cur_matched_icp, pose_error);
              t1 = std::chrono::steady_clock::now();
              icpTime.emplace_back(t1 - t0);
            }

            float pose_error_scaled = pose_error;
            if (direct_succ || icp_succ) {
              if (direct_succ) {
                directLoopCount++;
                tfm_cur_matched = tfm_cur_matched_direct;
                pose_error_scaled *= DIRECT_ERROR_SCALE;
                printf("            add loop");
              } else {
                icpLoopCount++;
                tfm_cur_matched = tfm_cur_matched_icp;
                pose_error_scaled *= ICP_ERROR_SCALE;
                printf("add loop");
              }

              // add the loop constraint
              cur_frame->edges.emplace_back(new LoopEdge(
                  matched_frame->kf_id,
                  g2o::SE3Quat(tfm_cur_matched.block<3, 3>(0, 0),
                               tfm_cur_matched.block<3, 1>(0, 3)),
                  pose_error_scaled,
                  SCALE_ERROR_SCALE * matched_frame->fh->scale_error));

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
            } // if pose estimate works
            printf("\n");
          } // if ScanContext search works
        }   // if ringkey search works
      }     // if signature is generated
      if (pangolinViewer) {
        static int prev_id = -1;
        if (cur_frame->kf_id > prev_id) {
          // for (auto &pt : lidar_pts) {
          //   pt = cur_frame->tfm_sc_rig * pt;
          // }
          pangolinViewer->refreshLidarData(lidar_pts, cur_frame->pts_sc.size());
          prev_id = cur_frame->kf_id;
        }
      }
    }
    delete cur_frame->fh;
    usleep(5000);
  }
  std::cout << "Finished Loop Thread" << std::endl;
}

} // namespace IOWrap
} // namespace dso