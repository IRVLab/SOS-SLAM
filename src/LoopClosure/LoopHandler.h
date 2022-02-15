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

#pragma once
#include <boost/thread.hpp>
#include <chrono>
#include <flann/flann.hpp>
#include <queue>

#include "g2o/core/block_solver.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/Pangolin/PangolinSOSVIOViewer.h"
#include "LoopClosure/PoseEstimator.h"
#include "LoopClosure/ScanContext.h"

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

typedef std::vector<std::chrono::duration<long int, std::ratio<1, 1000000000>>>
    TimeVector;

// normalize dso errors to roughly around 1.0
#define DSO_ERROR_SCALE 5.0
#define SCALE_ERROR_SCALE 0.1
#define DIRECT_ERROR_SCALE 0.1
#define ICP_ERROR_SCALE 1.0

// the rotation estimated by DSO is much more accurate than translation
#define POSE_R_WEIGHT 1e4

namespace dso {

class FrameHessian;
class CalibHessian;
class FrameShell;

class ScanContext;
class PoseEstimator;

struct LoopEdge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int id_from;
  g2o::SE3Quat measurement;
  Mat66 information;

  LoopEdge(int i, g2o::SE3Quat tfm_t_f, float pose_error, float scale_error)
      : id_from(i), measurement(tfm_t_f) {
    information.setIdentity();
    information *= (1.0 / pose_error);
    information.topLeftCorner<3, 3>() *=
        scale_error > 0 ? (1.0 / scale_error) : 1e-9;
    information.bottomRightCorner<3, 3>() *= POSE_R_WEIGHT;
  }
};

struct LoopFrame {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int kf_id;                     // kF id, for pose graph and visualization
  int incoming_id;               // increasing id, for ground truth
  g2o::SE3Quat tfm_w_c;          // coordinate in pose graph
  std::vector<LoopEdge *> edges; // pose edges associated with current frame

  // loop detection
  std::vector<std::pair<int, double>> signature; // place signature
  g2o::SE3Quat tfm_sc_rig; // transformation from rig to alignment frame

  // loop correction by dso
  std::vector<std::pair<Eigen::Vector3d, float *>> pts_dso;
  FrameHessian *fh;
  std::vector<float> cam;
  float ab_exposure;

  // loop correction by icp
  std::vector<Eigen::Vector3d> pts_sc;

  // whether has been added to global pose graph
  bool graph_added;

  LoopFrame(FrameHessian *fh, const std::vector<float> &cam,
            const g2o::SE3Quat &tsc,
            const std::vector<std::pair<Eigen::Vector3d, float *>> &pd,
            const std::vector<Eigen::Vector3d> &ps)
      : fh(fh), pts_dso(pd), kf_id(fh->frameID), tfm_sc_rig(tsc),
        incoming_id(fh->shell->incoming_id),
        tfm_w_c(g2o::SE3Quat(fh->shell->camToWorldScaled.rotationMatrix(),
                             fh->shell->camToWorldScaled.translation())),
        cam(cam), ab_exposure(fh->ab_exposure), pts_sc(ps), graph_added(false) {
  }

  ~LoopFrame() {
    for (auto &edge : edges) {
      delete edge;
    }

    for (auto &p : pts_dso) {
      delete p.second;
    }
  }
};

namespace IOWrap {

class LoopHandler : public Output3DWrapper {
public:
  LoopHandler(IOWrap::PangolinSOSVIOViewer *pangolin_viewer);
  ~LoopHandler();

  void publishKeyframes(std::vector<FrameHessian *> &frames, bool final,
                        CalibHessian *HCalib) override;
  void join();

  // statistics
  TimeVector scGenerationTime;
  TimeVector searchRingkeyTime;
  TimeVector searchScTime;
  TimeVector directEstTime;
  TimeVector icpTime;
  TimeVector optTime;
  int directLoopCount;
  int icpLoopCount;

private:
  bool running;
  boost::thread runThread;
  void run();

  void savePose();

  // loop detection by ScanContext
  flann::Index<flann::L2<float>> *ringkeys;
  ScanContext *scPtr;

  // loop correction by direct alignment
  PoseEstimator *poseEstimator;

  // pose graph
  boost::mutex loopFrameQueueMutex;
  std::queue<LoopFrame *> loopFrameQueue;
  std::vector<LoopFrame *> loopFrames;
  g2o::SparseOptimizer poseOptimizer;
  IOWrap::PangolinSOSVIOViewer *pangolinViewer;
  void optimize();

  // ROS pose publishers
  ros::NodeHandle rosNode;
  ros::Publisher currentPosePublisher;
  ros::Publisher marginalizedPosePublisher;

  // variables for forward-facing camera only
  int curFrameID;
};

} // namespace IOWrap

} // namespace dso
