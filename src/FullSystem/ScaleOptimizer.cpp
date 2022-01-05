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

#include "ScaleOptimizer.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/Residuals.h"
#include "IOWrapper/ImageRW.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#if !defined(__SSE3__) && !defined(__SSE2__) && !defined(__SSE1__)
#include "SSE2NEON.h"
#endif

#define DEBUG_PRINT false
#define DEBUG_PLOT false

namespace dso {

ScaleOptimizer::ScaleOptimizer(int ww, int hh,
                               const std::vector<double> &tfm_vec,
                               const Mat33f &K1) {
  for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
    int wl = ww >> lvl;
    int hl = hh >> lvl;

    idepth[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
    weight_sums[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
    weight_sums_bak[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);

    pc_u[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
    pc_v[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
    pc_idepth[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
    pc_color[lvl] = allocAligned<4, float>(wl * hl, ptrToDelete);
  }

  w[0] = h[0] = 0;

  // tranformation form frame0 to frame1
  Eigen::Matrix4d tfm_eigen;
  cv::Mat tfm_stereo_cv = cv::Mat(tfm_vec);
  tfm_stereo_cv = tfm_stereo_cv.reshape(0, 4);
  cv::cv2eigen(tfm_stereo_cv, tfm_eigen);
  tfmF0ToF1 = SE3(tfm_eigen);

  // make camera1 parameters
  fx1[0] = K1(0, 0);
  fy1[0] = K1(1, 1);
  cx1[0] = K1(0, 2);
  cy1[0] = K1(1, 2);
  for (int level = 1; level < pyrLevelsUsed; ++level) {
    fx1[level] = fx1[level - 1] * 0.5;
    fy1[level] = fy1[level - 1] * 0.5;
    cx1[level] = (cx1[0] + 0.5) / ((int)1 << level) - 0.5;
    cy1[level] = (cy1[0] + 0.5) / ((int)1 << level) - 0.5;
  }

  // scale warped buffers
  scaleBufWarped_rx1 = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_rx2 = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_rx3 = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_dx = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_dy = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_residual = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_weight = allocAligned<4, float>(ww * hh, ptrToDelete);
  scaleBufWarped_ref_color = allocAligned<4, float>(ww * hh, ptrToDelete);
}

ScaleOptimizer::~ScaleOptimizer() {
  for (float *ptr : ptrToDelete)
    delete[] ptr;
  ptrToDelete.clear();
}

void ScaleOptimizer::makeK(CalibHessian *HCalib) {
  w[0] = wG[0];
  h[0] = hG[0];

  fx[0] = HCalib->fxl();
  fy[0] = HCalib->fyl();
  cx[0] = HCalib->cxl();
  cy[0] = HCalib->cyl();

  for (int level = 1; level < pyrLevelsUsed; ++level) {
    w[level] = w[0] >> level;
    h[level] = h[0] >> level;
    fx[level] = fx[level - 1] * 0.5;
    fy[level] = fy[level - 1] * 0.5;
    cx[level] = (cx[0] + 0.5) / ((int)1 << level) - 0.5;
    cy[level] = (cy[0] + 0.5) / ((int)1 << level) - 0.5;
  }

  for (int level = 0; level < pyrLevelsUsed; ++level) {
    Mat33f K;
    K << fx[level], 0.0, cx[level], 0.0, fy[level], cy[level], 0.0, 0.0, 1.0;
    Ki[level] = K.inverse();
  }
}

float ScaleOptimizer::optimizeScale(FrameHessian *fh1, float &scale,
                                    int coarsestLvl) {
  assert(coarsestLvl < 5 && coarsestLvl < pyrLevelsUsed);
  fhStereo = fh1;

  Vec5 last_residuals;
  last_residuals.setConstant(NAN);

  int maxIterations[] = {10, 20, 50, 50, 50};
  float lambdaExtrapolationLimit = 0.001;

  float scale_current = scale;

  bool haveRepeated = false;

  for (int lvl = coarsestLvl; lvl >= 0; lvl--) {
    float H;
    float b;
    float levelCutoffRepeat = 1;
    Vec6 resOld = calcResScale(lvl, scale_current,
                               setting_coarseCutoffTH * levelCutoffRepeat);
    while (resOld[5] > 0.6 && levelCutoffRepeat < 50) {
      levelCutoffRepeat *= 2;
      resOld = calcResScale(lvl, scale_current,
                            setting_coarseCutoffTH * levelCutoffRepeat);

      if (!setting_debugout_runquiet)
        printf("INCREASING cutoff to %f (ratio is %f)!\n",
               setting_coarseCutoffTH * levelCutoffRepeat, resOld[5]);
    }

    calcGSSSEScale(lvl, H, b, scale_current);

    float lambda = 0.01;

    if (DEBUG_PRINT) {
      printf(
          "lvl%d, it %d (l=%f / %f) %s: %.3f->%.3f (%d -> %d) (inc = %f)! \n",
          lvl, -1, lambda, 1.0f, "INITIA", 0.0f, resOld[0] / resOld[1], 0,
          (int)resOld[1], 0.0f);
      std::cout << " Current scale " << scale_current << std::endl;
    }

    for (int iteration = 0; iteration < maxIterations[lvl]; iteration++) {
      float Hl = H;
      Hl *= (1 + lambda);
      float inc = -b / Hl;

      float extrapFac = 1;
      if (lambda < lambdaExtrapolationLimit)
        extrapFac = sqrt(sqrt(lambdaExtrapolationLimit / lambda));
      inc *= extrapFac;

      if (!std::isfinite(inc) || fabs(inc) > scale_current)
        inc = 0.0;

      float scale_new = scale_current + inc;

      Vec6 resNew = calcResScale(lvl, scale_new,
                                 setting_coarseCutoffTH * levelCutoffRepeat);

      bool accept = (resNew[0] / resNew[1]) < (resOld[0] / resOld[1]);

      if (DEBUG_PRINT) {
        printf("lvl %d, it %d (l=%f / %f) %s: %.3f->%.3f (%d -> %d) (inc = "
               "%f)! \t",
               lvl, iteration, lambda, extrapFac,
               (accept ? "ACCEPT" : "REJECT"), resOld[0] / resOld[1],
               resNew[0] / resNew[1], (int)resOld[1], (int)resNew[1], inc);
        std::cout << " New scale " << scale_new << std::endl;
      }

      if (accept) {
        calcGSSSEScale(lvl, H, b, scale_new);
        resOld = resNew;
        scale_current = scale_new;
        lambda *= 0.5;
      } else {
        lambda *= 4;
        if (lambda < lambdaExtrapolationLimit)
          lambda = lambdaExtrapolationLimit;
      }

      if (!(inc > 1e-3)) {
        if (DEBUG_PRINT)
          printf("inc too small, break!\n");
        break;
      }
    }

    // set last residual for that level, as well as flow indicators.
    last_residuals[lvl] = sqrtf((float)(resOld[0] / resOld[1]));

    if (levelCutoffRepeat > 1 && !haveRepeated) {
      lvl++;
      haveRepeated = true;
    }
  }

  // set!
  scale = scale_current;

  if (DEBUG_PLOT) {
    printf("scale = %.2f\n", scale);
    // calcResScale(1, scale / 10, setting_coarseCutoffTH, true);
    calcResScale(1, scale, setting_coarseCutoffTH, true);
    // calcResScale(1, scale * 10, setting_coarseCutoffTH, true);
  }

  return last_residuals[0];
}

void ScaleOptimizer::calcGSSSEScale(int lvl, float &H_out, float &b_out,
                                    float scale) {
  scaleAcc.initialize();

  __m128 fx1l = _mm_set1_ps(fx1[lvl]);
  __m128 fy1l = _mm_set1_ps(fy1[lvl]);

  __m128 s = _mm_set1_ps(scale);
  __m128 tx = _mm_set1_ps(tfmF0ToF1.translation()[0]);
  __m128 ty = _mm_set1_ps(tfmF0ToF1.translation()[1]);
  __m128 tz = _mm_set1_ps(tfmF0ToF1.translation()[2]);

  __m128 one = _mm_set1_ps(1);

  int n = scaleBufWarped_n;
  assert(n % 4 == 0);
  for (int i = 0; i < n; i += 4) {
    __m128 dxfx = _mm_mul_ps(_mm_load_ps(scaleBufWarped_dx + i), fx1l);
    __m128 dyfy = _mm_mul_ps(_mm_load_ps(scaleBufWarped_dy + i), fy1l);
    __m128 rx1 = _mm_load_ps(scaleBufWarped_rx1 + i);
    __m128 rx2 = _mm_load_ps(scaleBufWarped_rx2 + i);
    __m128 rx3 = _mm_load_ps(scaleBufWarped_rx3 + i);

    __m128 deno_sqrt = _mm_add_ps(_mm_mul_ps(s, rx3), tz);
    __m128 deno = _mm_div_ps(one, _mm_mul_ps(deno_sqrt, deno_sqrt));

    __m128 xno = _mm_sub_ps(_mm_mul_ps(rx1, tz), _mm_mul_ps(rx3, tx));
    __m128 yno = _mm_sub_ps(_mm_mul_ps(rx2, tz), _mm_mul_ps(rx3, ty));

    scaleAcc.updateSSE_oneed(
        _mm_add_ps(_mm_mul_ps(dxfx, _mm_mul_ps(deno, xno)),
                   _mm_mul_ps(dyfy, _mm_mul_ps(deno, yno))),
        _mm_load_ps(scaleBufWarped_residual + i),
        _mm_load_ps(scaleBufWarped_weight + i));
  }

  scaleAcc.finish();
  H_out = scaleAcc.hessian(0, 0) * (1.0f / n);
  b_out = scaleAcc.hessian(0, 1) * (1.0f / n);
}

Vec6 ScaleOptimizer::calcResScale(int lvl, float scale, float cutoffTH,
                                  bool plot_img) {
  float E = 0;
  int numTermsInE = 0;
  int numTermsInWarped = 0;
  int numSaturated = 0;

  int wl = w[lvl];
  int hl = h[lvl];
  Eigen::Vector3f *dINewl = fhStereo->dIp[lvl];
  float fx1l = fx1[lvl];
  float fy1l = fy1[lvl];
  float cx1l = cx1[lvl];
  float cy1l = cy1[lvl];

  Mat33f rot_f1_f0_K0_i = (tfmF0ToF1.rotationMatrix().cast<float>() * Ki[lvl]);
  Vec3f tsl_f1_f0 = (tfmF0ToF1.translation()).cast<float>();

  float sumSquaredShiftT = 0;
  float sumSquaredShiftRT = 0;
  float sumSquaredShiftNum = 0;

  float maxEnergy =
      2 * setting_huberTH * cutoffTH -
      setting_huberTH * setting_huberTH; // energy for r=setting_coarseCutoffTH.

  MinimalImageB3 *resImage = 0;
  MinimalImageB3 *projImage = 0;
  if (plot_img) {
    resImage = new MinimalImageB3(wl, hl);
    resImage->setConst(Vec3b(255, 255, 255));

    projImage = new MinimalImageB3(wl, hl);
    projImage->setBlack();
    for (int i = 0; i < h[lvl] * w[lvl]; i++) {
      int c = fhStereo->dIp[lvl][i][0] * 0.9f;
      if (c > 255)
        c = 255;
      projImage->at(i) = Vec3b(c, c, c);
    }
  }

  int nl = pc_n[lvl];
  float *lpc_u = pc_u[lvl];
  float *lpc_v = pc_v[lvl];
  float *lpc_idepth = pc_idepth[lvl];
  float *lpc_color = pc_color[lvl];

  for (int i = 0; i < nl; i++) {
    float id = lpc_idepth[i];
    float x = lpc_u[i];
    float y = lpc_v[i];

    Vec3f pt = scale * rot_f1_f0_K0_i * Vec3f(x, y, 1) + tsl_f1_f0 * id;
    float u = pt[0] / pt[2];
    float v = pt[1] / pt[2];
    float Ku = fx1l * u + cx1l;
    float Kv = fy1l * v + cy1l;
    float new_idepth = id / pt[2];

    Vec3f rx = rot_f1_f0_K0_i * Vec3f(x, y, 1) / id;

    if (lvl == 0 && i % 32 == 0) {
      // translation only (positive)
      Vec3f ptT = scale * Ki[lvl] * Vec3f(x, y, 1) + tsl_f1_f0 * id;
      float uT = ptT[0] / ptT[2];
      float vT = ptT[1] / ptT[2];
      float KuT = fx1l * uT + cx1l;
      float KvT = fy1l * vT + cy1l;

      // translation only (negative)
      Vec3f ptT2 = scale * Ki[lvl] * Vec3f(x, y, 1) - tsl_f1_f0 * id;
      float uT2 = ptT2[0] / ptT2[2];
      float vT2 = ptT2[1] / ptT2[2];
      float KuT2 = fx1l * uT2 + cx1l;
      float KvT2 = fy1l * vT2 + cy1l;

      // translation and rotation (negative)
      Vec3f pt3 = scale * rot_f1_f0_K0_i * Vec3f(x, y, 1) - tsl_f1_f0 * id;
      float u3 = pt3[0] / pt3[2];
      float v3 = pt3[1] / pt3[2];
      float Ku3 = fx1l * u3 + cx1l;
      float Kv3 = fy1l * v3 + cy1l;

      // translation and rotation (positive)
      // already have it.

      sumSquaredShiftT += (KuT - x) * (KuT - x) + (KvT - y) * (KvT - y);
      sumSquaredShiftT += (KuT2 - x) * (KuT2 - x) + (KvT2 - y) * (KvT2 - y);
      sumSquaredShiftRT += (Ku - x) * (Ku - x) + (Kv - y) * (Kv - y);
      sumSquaredShiftRT += (Ku3 - x) * (Ku3 - x) + (Kv3 - y) * (Kv3 - y);
      sumSquaredShiftNum += 2;
    }

    if (!(Ku > 2 && Kv > 2 && Ku < wl - 3 && Kv < hl - 3 && new_idepth > 0))
      continue;

    float refColor = lpc_color[i];
    Vec3f hitColor = getInterpolatedElement33(dINewl, Ku, Kv, wl);
    if (!std::isfinite((float)hitColor[0]))
      continue;
    float residual = hitColor[0] - refColor;
    float hw =
        fabs(residual) < setting_huberTH ? 1 : setting_huberTH / fabs(residual);

    if (plot_img)
      projImage->setPixel4(Ku, Kv, Vec3b(refColor, refColor, refColor));

    if (fabs(residual) > cutoffTH) {
      if (plot_img)
        resImage->setPixel4(lpc_u[i], lpc_v[i], Vec3b(0, 0, 255));
      E += maxEnergy;
      numTermsInE++;
      numSaturated++;
    } else {
      if (plot_img)
        resImage->setPixel4(
            lpc_u[i], lpc_v[i],
            Vec3b(residual + 128, residual + 128, residual + 128));
      E += hw * residual * residual * (2 - hw);
      numTermsInE++;

      scaleBufWarped_rx1[numTermsInWarped] = rx[0];
      scaleBufWarped_rx2[numTermsInWarped] = rx[1];
      scaleBufWarped_rx3[numTermsInWarped] = rx[2];
      scaleBufWarped_dx[numTermsInWarped] = hitColor[1];
      scaleBufWarped_dy[numTermsInWarped] = hitColor[2];
      scaleBufWarped_residual[numTermsInWarped] = residual;
      scaleBufWarped_weight[numTermsInWarped] = hw;
      scaleBufWarped_ref_color[numTermsInWarped] = lpc_color[i];
      numTermsInWarped++;
    }
  }

  while (numTermsInWarped % 4 != 0) {
    scaleBufWarped_rx1[numTermsInWarped] = 0;
    scaleBufWarped_rx2[numTermsInWarped] = 0;
    scaleBufWarped_rx3[numTermsInWarped] = 0;
    scaleBufWarped_dx[numTermsInWarped] = 0;
    scaleBufWarped_dy[numTermsInWarped] = 0;
    scaleBufWarped_residual[numTermsInWarped] = 0;
    scaleBufWarped_weight[numTermsInWarped] = 0;
    scaleBufWarped_ref_color[numTermsInWarped] = 0;
    numTermsInWarped++;
  }
  scaleBufWarped_n = numTermsInWarped;

  if (plot_img) {
    IOWrap::displayImage("Scale Residual", resImage, false);
    IOWrap::displayImage("Projection", projImage, false);
    IOWrap::waitKey(0);
    delete resImage;
    delete projImage;
  }

  Vec6 rs;
  rs[0] = E;
  rs[1] = numTermsInE;
  rs[2] = sumSquaredShiftT / (sumSquaredShiftNum + 0.1);
  rs[3] = 0;
  rs[4] = sumSquaredShiftRT / (sumSquaredShiftNum + 0.1);
  rs[5] = numSaturated / (float)numTermsInE;

  return rs;
}

} // namespace dso
