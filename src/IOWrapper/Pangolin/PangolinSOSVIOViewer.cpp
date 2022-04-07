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

#include "PangolinSOSVIOViewer.h"
#include "KeyFrameDisplay.h"

#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/globalCalib.h"
#include "util/settings.h"

namespace dso {
namespace IOWrap {

PangolinSOSVIOViewer::PangolinSOSVIOViewer(int w_, int h_,
                                           bool startRunThread) {
  w = w_;
  h = h_;
  running = true;

  boost::unique_lock<boost::mutex> lk(openImagesMutex);

  internalVideoImg = new MinimalImageB3(w, h);
  internalVideoImg->setBlack();

  internalKFImg = new MinimalImageB3(w, h);
  internalKFImg->setBlack();

  setting_render_renderWindowFrames = false;
  setting_render_plotTrackingFull = false;
  setting_render_displayCoarseTrackingFull = false;

  if (startRunThread)
    runThread = boost::thread(&PangolinSOSVIOViewer::run, this);

  lidarCurSize = 0;

  imuBa = -1;
  imuBg = -1;
}

PangolinSOSVIOViewer::~PangolinSOSVIOViewer() {
  close();
  runThread.join();
}

void PangolinSOSVIOViewer::run() {
  std::string window_name = "SOS-SLAM: ScaleOpt ";
  window_name += setting_enable_scale_opt ? "enabled" : "disabled";
  window_name += "; IMU ";
  window_name += setting_enable_imu ? "enabled" : "disabled";
  window_name += "; Loop ";
  window_name += setting_enable_loop_closure ? "enabled" : "disabled";

  pangolin::CreateWindowAndBind(window_name, 960, 1080);
  const float ratio = w / float(h);

  auto proj_mat =
      pangolin::ProjectionMatrix(w, h, 200, 200, w / 2, h / 2, 0.1, 1000);
  auto model_view =
      pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY);

  glEnable(GL_DEPTH_TEST);

  // 3D visualization
  pangolin::OpenGlRenderState Visualization3D_camera(proj_mat, model_view);

  pangolin::View &Visualization3D_display =
      pangolin::CreateDisplay()
          .SetBounds(0.3, 1.0, 0.0, 1.0, -ratio)
          .SetHandler(new pangolin::Handler3D(Visualization3D_camera));

  // keyframe depth visualization
  pangolin::GlTexture texVideo(w, h, GL_RGB, false, 0, GL_RGB,
                               GL_UNSIGNED_BYTE);
  pangolin::View &d_video =
      pangolin::Display("imgVideo").SetAspect(w / (float)h);

  pangolin::GlTexture texKFDepth(w, h, GL_RGB, false, 0, GL_RGB,
                                 GL_UNSIGNED_BYTE);
  pangolin::View &d_kfDepth = pangolin::Display("imgKFDepth").SetAspect(ratio);

  // lidar visualization
  pangolin::OpenGlRenderState Visualization_lidar_camera(proj_mat, model_view);
  pangolin::View &Visualization_lidar_display =
      pangolin::Display("lidarDisplay")
          .SetAspect(ratio)
          .SetHandler(new pangolin::Handler3D(Visualization_lidar_camera));

  pangolin::CreateDisplay()
      .SetBounds(pangolin::Attach::Pix(25), 0.3, 0.0, 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal)
      .AddDisplay(d_video)
      .AddDisplay(d_kfDepth)
      .AddDisplay(Visualization_lidar_display);

  pangolin::CreatePanel("ui")
      .SetBounds(0.0, pangolin::Attach::Pix(20), 0.0, 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal);

  pangolin::Var<int> settings_frameID("ui.frame", 0, 0, 0, false);
  pangolin::Var<double> settings_scaleScale("ui.|scale", 0, 0, 0, false);
  pangolin::Var<double> settings_scaleErrInit(
      (setting_enable_scale_opt ? "ui.|scale_err" : "ui.|scale_init"), 0, 0, 0,
      false);
  pangolin::Var<double> settings_imuBa("ui.|IMU_ba", 0, 0, 0, false);
  pangolin::Var<double> settings_imuBg("ui.|IMU_bg", 0, 0, 0, false);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while (!pangolin::ShouldQuit() && running) {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // Activate efficiently by object
    Visualization3D_display.Activate(Visualization3D_camera);
    boost::unique_lock<boost::mutex> lk3d(model3dMutex);
    // pangolin::glDrawColouredCube();
    for (auto &id_fh : keyframesById) {
      id_fh.second->refreshPC();
      id_fh.second->drawPC(1);
    }
    if (!keyframesIdSorted.empty()) {
      keyframesById[keyframesIdSorted.back()]->drawCam(2, 0, 1);
    }
    drawConstraints();
    lk3d.unlock();

    // refresh loop closure visualization
    boost::unique_lock<boost::mutex> lkLoop(loopClosureMutex);
    drawLoopClosures();
    lkLoop.unlock();

    openImagesMutex.lock();
    if (videoImgChanged)
      texVideo.Upload(internalVideoImg->data, GL_BGR, GL_UNSIGNED_BYTE);
    videoImgChanged = false;
    if (KFImgChanged)
      texKFDepth.Upload(internalKFImg->data, GL_BGR, GL_UNSIGNED_BYTE);
    KFImgChanged = false;
    openImagesMutex.unlock();

    d_video.Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    texVideo.RenderToViewportFlipY();

    d_kfDepth.Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    texKFDepth.RenderToViewportFlipY();

    Visualization_lidar_display.Activate(Visualization_lidar_camera);
    boost::unique_lock<boost::mutex> lklidar(modelLidarMutex);
    drawLidar();
    lklidar.unlock();

    settings_frameID = frameID;
    settings_scaleScale = int(100 * scaleScale) / 100.0;
    settings_scaleErrInit = int(100 * scaleErrInit) / 100.0;
    settings_imuBa = int(100 * imuBa) / 100.0;
    settings_imuBg = int(100 * imuBg) / 100.0;

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  if (running) {
    exit(1);
  }
}

void PangolinSOSVIOViewer::close() { running = false; }

void PangolinSOSVIOViewer::join() {
  runThread.join();
  printf("JOINED Pangolin thread!\n");
}

void PangolinSOSVIOViewer::drawConstraints() {
  float colorRed[3] = {1, 0, 0};
  glColor3f(colorRed[0], colorRed[1], colorRed[2]);
  glLineWidth(3);

  glBegin(GL_LINE_STRIP);
  for (unsigned int i = 0; i < keyframesIdSorted.size(); i++) {
    glVertex3f(
        (float)keyframesById[keyframesIdSorted[i]]->tfmCToW.translation()[0],
        (float)keyframesById[keyframesIdSorted[i]]->tfmCToW.translation()[1],
        (float)keyframesById[keyframesIdSorted[i]]->tfmCToW.translation()[2]);
  }
  glEnd();
}

void PangolinSOSVIOViewer::drawLoopClosures() {
  float colorGreen[3] = {0, 1, 0};
  glColor3f(colorGreen[0], colorGreen[1], colorGreen[2]);
  glLineWidth(6);

  for (unsigned int i = 0; i < loopClosures.size(); i++) {
    glBegin(GL_LINE_STRIP);
    glVertex3f(
        (float)keyframesById[loopClosures[i].first]->tfmCToW.translation()[0],
        (float)keyframesById[loopClosures[i].first]->tfmCToW.translation()[1],
        (float)keyframesById[loopClosures[i].first]->tfmCToW.translation()[2]);
    glVertex3f(
        (float)keyframesById[loopClosures[i].second]->tfmCToW.translation()[0],
        (float)keyframesById[loopClosures[i].second]->tfmCToW.translation()[1],
        (float)keyframesById[loopClosures[i].second]->tfmCToW.translation()[2]);
    glEnd();
  }
}

void PangolinSOSVIOViewer::publishKeyframes(std::vector<FrameHessian *> &frames,
                                            bool final, CalibHessian *HCalib) {
  // only work on marginalized frame
  if (!final) {
    // update frame info
    frameID = frames.back()->shell->incoming_id;
    scaleScale = HCalib->getScaleScaled();
    scaleErrInit =
        setting_enable_scale_opt
            ? (frames.size() >= 2 ? frames[frames.size() - 2]->scale_error : -1)
            : HCalib->getScaleScaled(true);
    if (setting_enable_imu) {
      imuBa = frames.back()->imu_bias.head(3).norm();
      imuBg = frames.back()->imu_bias.tail(3).norm();
    }
    return;
  }

  assert(frames.size() == 1); // contains only one marginalized frame
  FrameHessian *fh = frames[0];

  static int prv_id = -1;
  // keep incoming id increasing
  if (setting_cam_mode == FORWARD_CAM && prv_id >= fh->frameID) {
    return;
  }
  prv_id = fh->frameID;

  boost::unique_lock<boost::mutex> lk(model3dMutex);
  if (keyframesById.find(fh->frameID) == keyframesById.end()) {
    keyframesById[fh->frameID] = new KeyFrameDisplay();
    keyframesIdSorted.push_back(fh->frameID);
    int i = keyframesIdSorted.size() - 2;
    while ((i >= 0) && (keyframesIdSorted[i] > keyframesIdSorted[i + 1])) {
      size_t tmp = keyframesIdSorted[i];
      keyframesIdSorted[i] = keyframesIdSorted[i + 1];
      keyframesIdSorted[i + 1] = tmp;
      i--;
    }
  }
  keyframesById[fh->frameID]->setFromKF(fh, HCalib);
}

void PangolinSOSVIOViewer::modifyKeyframePoseByKFID(int id,
                                                    const SE3 &poseCamToWorld) {
  boost::unique_lock<boost::mutex> lk3d(model3dMutex);
  keyframesById[id]->tfmCToW = poseCamToWorld;
  keyframesById[id]->needRefresh = true;
}

void PangolinSOSVIOViewer::refreshLidarData(
    const std::vector<Eigen::Vector3d> &pts, size_t cur_sz) {
  boost::unique_lock<boost::mutex> lk(modelLidarMutex);
  assert(cur_sz <= pts.size());
  lidarPts = pts;
  lidarCurSize = cur_sz;
}

void PangolinSOSVIOViewer::drawLidar() {
  glPointSize(3.0);

  glBegin(GL_POINTS);
  for (size_t i = 0; i < lidarPts.size(); i++) {
    if (i < lidarCurSize) {
      glColor3ub(0, 255, 0);
    } else {
      glColor3ub(255, 0, 0);
    }
    glVertex3f(lidarPts[i](0), lidarPts[i](1), lidarPts[i](2));
  }
  glEnd();
}

void PangolinSOSVIOViewer::pushLoopClosure(
    std::pair<size_t, size_t> new_loop_pair) {
  boost::unique_lock<boost::mutex> lk(loopClosureMutex);
  loopClosures.push_back(new_loop_pair);
}

void PangolinSOSVIOViewer::pushLiveFrame(FrameHessian *image) {
  boost::unique_lock<boost::mutex> lk(openImagesMutex);

  for (int i = 0; i < w * h; i++)
    internalVideoImg->data[i][0] = internalVideoImg->data[i][1] =
        internalVideoImg->data[i][2] =
            image->dI[i][0] * 0.8 > 255.0f ? 255.0 : image->dI[i][0] * 0.8;

  videoImgChanged = true;
}

void PangolinSOSVIOViewer::pushDepthImage(MinimalImageB3 *image) {

  if (!setting_render_displayDepth)
    return;
  if (disableAllDisplay)
    return;

  boost::unique_lock<boost::mutex> lk(openImagesMutex);
  memcpy(internalKFImg->data, image->data, w * h * 3);
  KFImgChanged = true;
}

} // namespace IOWrap
} // namespace dso
