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

#include "util/settings.h"
#include <stdio.h>

//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "KeyFrameDisplay.h"
#include "util/FrameShell.h"

namespace dso {
namespace IOWrap {

KeyFrameDisplay::KeyFrameDisplay() {
  originalInputSparse = 0;
  numSparseBufferSize = 0;
  numSparsePoints = 0;

  id = 0;
  active = true;
  tfmCToW = SE3();

  needRefresh = true;

  myScaledTh = 0.001;
  myAbsTh = 0.001;
  myDisplayMode = 1;
  myMinRelBs = 0.1;
  mySparsifyFactor = 1;

  numGlBufferPoints = 0;
  bufferValid = false;
}
void KeyFrameDisplay::setFromF(FrameShell *frame, CalibHessian *HCalib) {
  id = frame->id;
  fx = HCalib->fxl();
  fy = HCalib->fyl();
  cx = HCalib->cxl();
  cy = HCalib->cyl();
  width = wG[0];
  height = hG[0];
  fxi = 1 / fx;
  fyi = 1 / fy;
  cxi = -cx / fx;
  cyi = -cy / fy;
  tfmCToW = frame->camToWorld;
  needRefresh = true;
}

void KeyFrameDisplay::setFromKF(FrameHessian *fh, CalibHessian *HCalib) {
  setFromF(fh->shell, HCalib);

  // add all traces, inlier and outlier points.
  int npoints = fh->immaturePoints.size() + fh->pointHessians.size() +
                fh->pointHessiansMarginalized.size() +
                fh->pointHessiansOut.size();

  if (numSparseBufferSize < npoints) {
    if (originalInputSparse != 0)
      delete originalInputSparse;
    numSparseBufferSize = npoints + 100;
    originalInputSparse =
        new InputPointSparse<MAX_RES_PER_POINT>[numSparseBufferSize];
  }

  InputPointSparse<MAX_RES_PER_POINT> *pc = originalInputSparse;
  numSparsePoints = 0;
  for (ImmaturePoint *p : fh->immaturePoints) {
    for (int i = 0; i < patternNum; i++)
      pc[numSparsePoints].color[i] = p->color[i];

    pc[numSparsePoints].u = p->u;
    pc[numSparsePoints].v = p->v;
    pc[numSparsePoints].idpeth = (p->idepth_max + p->idepth_min) * 0.5f;
    pc[numSparsePoints].idepth_hessian = 1000;
    pc[numSparsePoints].relObsBaseline = 0;
    pc[numSparsePoints].numGoodRes = 1;
    pc[numSparsePoints].status = 0;
    numSparsePoints++;
  }

  for (PointHessian *p : fh->pointHessians) {
    for (int i = 0; i < patternNum; i++)
      pc[numSparsePoints].color[i] = p->color[i];
    pc[numSparsePoints].u = p->u;
    pc[numSparsePoints].v = p->v;
    pc[numSparsePoints].idpeth = p->idepth_scaled;
    pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
    pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
    pc[numSparsePoints].numGoodRes = 0;
    pc[numSparsePoints].status = 1;

    numSparsePoints++;
  }

  for (PointHessian *p : fh->pointHessiansMarginalized) {
    for (int i = 0; i < patternNum; i++)
      pc[numSparsePoints].color[i] = p->color[i];
    pc[numSparsePoints].u = p->u;
    pc[numSparsePoints].v = p->v;
    pc[numSparsePoints].idpeth = p->idepth_scaled;
    pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
    pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
    pc[numSparsePoints].numGoodRes = 0;
    pc[numSparsePoints].status = 2;
    numSparsePoints++;
  }

  for (PointHessian *p : fh->pointHessiansOut) {
    for (int i = 0; i < patternNum; i++)
      pc[numSparsePoints].color[i] = p->color[i];
    pc[numSparsePoints].u = p->u;
    pc[numSparsePoints].v = p->v;
    pc[numSparsePoints].idpeth = p->idepth_scaled;
    pc[numSparsePoints].relObsBaseline = p->maxRelBaseline;
    pc[numSparsePoints].idepth_hessian = p->idepth_hessian;
    pc[numSparsePoints].numGoodRes = 0;
    pc[numSparsePoints].status = 3;
    numSparsePoints++;
  }
  assert(numSparsePoints <= npoints);

  tfmCToW = fh->PRE_camToWorld;
  needRefresh = true;
}

KeyFrameDisplay::~KeyFrameDisplay() {
  if (originalInputSparse != 0)
    delete[] originalInputSparse;
}

void KeyFrameDisplay::refreshPC() {
  if (!needRefresh)
    return;
  needRefresh = false;

  // if there are no vertices, done!
  if (numSparsePoints == 0)
    return;

  // make data
  Vec3f *tmpVertexBuffer = new Vec3f[numSparsePoints * patternNum];
  Vec3b *tmpColorBuffer = new Vec3b[numSparsePoints * patternNum];
  int vertexBufferNumPoints = 0;

  for (int i = 0; i < numSparsePoints; i++) {
    /* display modes:
     * myDisplayMode==0 - all pts, color-coded
     * myDisplayMode==1 - normal points
     * myDisplayMode==2 - active only
     * myDisplayMode==3 - nothing
     */

    if (myDisplayMode == 1 && originalInputSparse[i].status != 1 &&
        originalInputSparse[i].status != 2)
      continue;
    if (myDisplayMode == 2 && originalInputSparse[i].status != 1)
      continue;
    if (myDisplayMode > 2)
      continue;

    if (originalInputSparse[i].idpeth < 0)
      continue;

    float depth = 1.0f / originalInputSparse[i].idpeth;
    float depth4 = depth * depth;
    depth4 *= depth4;
    float var = (1.0f / (originalInputSparse[i].idepth_hessian + 0.01));

    if (var * depth4 > myScaledTh)
      continue;

    if (var > myAbsTh)
      continue;

    if (originalInputSparse[i].relObsBaseline < myMinRelBs)
      continue;

    for (int pnt = 0; pnt < patternNum; pnt++) {

      if (mySparsifyFactor > 1 && rand() % mySparsifyFactor != 0)
        continue;
      int dx = patternP[pnt][0];
      int dy = patternP[pnt][1];

      tmpVertexBuffer[vertexBufferNumPoints][0] =
          ((originalInputSparse[i].u + dx) * fxi + cxi) * depth;
      tmpVertexBuffer[vertexBufferNumPoints][1] =
          ((originalInputSparse[i].v + dy) * fyi + cyi) * depth;
      tmpVertexBuffer[vertexBufferNumPoints][2] =
          depth * (1 + 2 * fxi * (rand() / (float)RAND_MAX - 0.5f));

      if (myDisplayMode == 0) {
        if (originalInputSparse[i].status == 0) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        } else if (originalInputSparse[i].status == 1) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 0;
        } else if (originalInputSparse[i].status == 2) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 0;
          tmpColorBuffer[vertexBufferNumPoints][1] = 0;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        } else if (originalInputSparse[i].status == 3) {
          tmpColorBuffer[vertexBufferNumPoints][0] = 255;
          tmpColorBuffer[vertexBufferNumPoints][1] = 0;
          tmpColorBuffer[vertexBufferNumPoints][2] = 0;
        } else {
          tmpColorBuffer[vertexBufferNumPoints][0] = 255;
          tmpColorBuffer[vertexBufferNumPoints][1] = 255;
          tmpColorBuffer[vertexBufferNumPoints][2] = 255;
        }

      } else {
        tmpColorBuffer[vertexBufferNumPoints][0] =
            originalInputSparse[i].color[pnt];
        tmpColorBuffer[vertexBufferNumPoints][1] =
            originalInputSparse[i].color[pnt];
        tmpColorBuffer[vertexBufferNumPoints][2] =
            originalInputSparse[i].color[pnt];
      }
      vertexBufferNumPoints++;

      assert(vertexBufferNumPoints <= numSparsePoints * patternNum);
    }
  }

  if (vertexBufferNumPoints == 0) {
    delete[] tmpColorBuffer;
    delete[] tmpVertexBuffer;
    return;
  }

  numGlBufferGoodPoints = vertexBufferNumPoints;
  if (numGlBufferGoodPoints > numGlBufferPoints) {
    numGlBufferPoints = vertexBufferNumPoints * 1.3;
    vertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numGlBufferPoints,
                                GL_FLOAT, 3, GL_DYNAMIC_DRAW);
    colorBuffer.Reinitialise(pangolin::GlArrayBuffer, numGlBufferPoints,
                               GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
  }
  vertexBuffer.Upload(tmpVertexBuffer,
                        sizeof(float) * 3 * numGlBufferGoodPoints, 0);
  colorBuffer.Upload(tmpColorBuffer,
                       sizeof(unsigned char) * 3 * numGlBufferGoodPoints,
                       0);
  bufferValid = true;
  delete[] tmpColorBuffer;
  delete[] tmpVertexBuffer;
}

void KeyFrameDisplay::drawPC(float pointSize) {

  if (!bufferValid || numGlBufferGoodPoints == 0)
    return;

  glDisable(GL_LIGHTING);

  glPushMatrix();

  Sophus::Matrix4f m = tfmCToW.matrix().cast<float>();
  glMultMatrixf((GLfloat *)m.data());

  glPointSize(pointSize);

  colorBuffer.Bind();
  glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
  glEnableClientState(GL_COLOR_ARRAY);

  vertexBuffer.Bind();
  glVertexPointer(vertexBuffer.count_per_element, vertexBuffer.datatype, 0,
                  0);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDrawArrays(GL_POINTS, 0, numGlBufferGoodPoints);
  glDisableClientState(GL_VERTEX_ARRAY);
  vertexBuffer.Unbind();

  glDisableClientState(GL_COLOR_ARRAY);
  colorBuffer.Unbind();

  glPopMatrix();
}

} // namespace IOWrap
} // namespace dso
