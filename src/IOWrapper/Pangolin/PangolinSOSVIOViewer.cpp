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
  internalKFImg = new MinimalImageB3(w, h);
  internalKFImg->setBlack();

  setting_render_renderWindowFrames = false;
  setting_render_plotTrackingFull = false;
  setting_render_displayCoarseTrackingFull = false;

  if (startRunThread)
    runThread = boost::thread(&PangolinSOSVIOViewer::run, this);

  lidarCurSize = 0;
}

PangolinSOSVIOViewer::~PangolinSOSVIOViewer() {
  close();
  runThread.join();
}

void PangolinSOSVIOViewer::run() {
  pangolin::CreateWindowAndBind("SOS-SLAM", 960, 1080);
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
      .SetBounds(0.0, 0.3, 0.0, 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal)
      .AddDisplay(d_kfDepth)
      .AddDisplay(Visualization_lidar_display);

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while (!pangolin::ShouldQuit() && running) {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // Activate efficiently by object
    Visualization3D_display.Activate(Visualization3D_camera);
    boost::unique_lock<boost::mutex> lk3d(model3dMutex);
    // pangolin::glDrawColouredCube();
    for (KeyFrameDisplay *fh : keyframes) {
      fh->refreshPC();
      fh->drawPC(1);
    }
    drawConstraints();
    lk3d.unlock();

    openImagesMutex.lock();
    if (KFImgChanged)
      texKFDepth.Upload(internalKFImg->data, GL_BGR, GL_UNSIGNED_BYTE);
    KFImgChanged = false;
    openImagesMutex.unlock();

    d_kfDepth.Activate();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    texKFDepth.RenderToViewportFlipY();

    Visualization_lidar_display.Activate(Visualization_lidar_camera);
    boost::unique_lock<boost::mutex> lklidar(modelLidarMutex);
    drawLidar();
    lklidar.unlock();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  exit(1);
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
  for (unsigned int i = 0; i < keyframes.size(); i++) {
    glVertex3f((float)keyframes[i]->tfmCToW.translation()[0],
               (float)keyframes[i]->tfmCToW.translation()[1],
               (float)keyframes[i]->tfmCToW.translation()[2]);
  }
  glEnd();
}

void PangolinSOSVIOViewer::publishKeyframes(std::vector<FrameHessian *> &frames,
                                            bool final, CalibHessian *HCalib) {
  // only work on marginalized frame
  if (!final)
    return;

  assert(frames.size() == 1); // contains only one marginalized frame
  FrameHessian *fh = frames[0];

  static int prv_id = -1;

  // keep incoming id increasing
  if (prv_id >= fh->frameID) {
    return;
  }
  prv_id = fh->frameID;

  boost::unique_lock<boost::mutex> lk(model3dMutex);
  if (keyframesById.find(fh->frameID) == keyframesById.end()) {
    KeyFrameDisplay *kfd = new KeyFrameDisplay();
    keyframesById[fh->frameID] = kfd;
    keyframes.push_back(kfd);
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
