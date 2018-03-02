/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file PoseOptimizerSL.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include <boost/math/tools/minima.hpp>
#include "PoseOptimizerSL.h"

using namespace std;

////////

// データ対応づけ固定のもと、初期値initPoseを与えてロボット位置の推定値estPoseを求める
double PoseOptimizerSL::optimizePose(Pose2D &initPose, Pose2D &estPose) {
  double th = initPose.th;
  double tx = initPose.tx;
  double ty = initPose.ty;
  double txmin=tx, tymin=ty, thmin=th;           // コスト最小の解
  double evmin = HUGE_VAL;                       // コストの最小値
  double evold = evmin;                          // 1つ前のコスト値。収束判定に使う
  Pose2D pose, dir;

  double ev = cfunc->calValue(tx, ty, th);       // コスト計算
  int nn=0;                                      // 繰り返し回数。確認用
  while (abs(evold-ev) > evthre) {               // 収束判定。値の変化が小さいと終了
    nn++;
    evold = ev;

    // 数値計算による偏微分
    double dx = (cfunc->calValue(tx+dd, ty, th) - ev)/dd;
    double dy = (cfunc->calValue(tx, ty+dd, th) - ev)/dd;
    double dth = (cfunc->calValue(tx, ty, th+da) - ev)/da;
    tx += dx;  ty += dy;  th += dth;              // いったん次の探索位置を決める

    // ブレント法による直線探索
    pose.tx = tx;  pose.ty = ty;  pose.th = th;   // 探索開始点
    dir.tx = dx;   dir.ty = dy;   dir.th = dth;   // 探索方向
    search(ev, pose, dir);                        // 直線探索実行
    tx = pose.tx;  ty = pose.ty;  th = pose.th;   // 直線探索で求めた位置

    ev = cfunc->calValue(tx, ty, th);             // 求めた位置でコスト計算

    if (ev < evmin) {                             // コストがこれまでの最小なら更新
      evmin = ev;
      txmin = tx;  tymin = ty;  thmin = th;
    }

//    printf("nn=%d, ev=%g, evold=%g, abs(evold-ev)=%g\n", nn, ev, evold, abs(evold-ev));         // 確認用
  }
  ++allN;
  if (allN > 0 && evmin < 100) 
    sum += evmin;
//  printf("allN=%d, nn=%d, evmin=%g, avg=%g, evthre=%g\n", allN, nn, evmin, (sum/allN), evthre);         // 確認用

//  printf("nn=%d, evmin=%g\n", nn, evmin);       // 確認用

  estPose.setVal(txmin, tymin, thmin);            // 最小値を与える解を保存

  return(evmin);
}

////////// Line search ///////////

// boostライブラリのブレント法で直線探索を行う。
// poseを始点に、dp方向にどれだけ進めばよいかステップ幅を見つける。
double PoseOptimizerSL::search(double ev0, Pose2D &pose, Pose2D &dp) {
  int bits = numeric_limits<double>::digits;         // 探索精度
  boost::uintmax_t maxIter=40;                       // 最大繰り返し回数。経験的に決める
  pair<double, double> result = 
    boost::math::tools::brent_find_minima(
    [this, &pose, &dp](double tt) {return (objFunc(tt, pose, dp));},
    -2.0, 2.0, bits, maxIter);                       // 探索範囲(-2.0,2.0)

  double t = result.first;                           // 求めるステップ幅
  double v = result.second;                          // 求める最小値

  pose.tx = pose.tx + t*dp.tx;                       // 求める最小解をposeに格納
  pose.ty = pose.ty + t*dp.ty;
  pose.th = MyUtil::add(pose.th, t*dp.th);

  return(v);
}  

// 直線探索の目的関数。ttがステップ幅
double PoseOptimizerSL::objFunc(double tt, Pose2D &pose, Pose2D &dp) {
  double tx = pose.tx + tt*dp.tx;                     // poseからdp方向にttだけ進む
  double ty = pose.ty + tt*dp.ty;
  double th = MyUtil::add(pose.th, tt*dp.th);
  double v = cfunc->calValue(tx, ty, th);             // コスト関数値

  return(v);
}
