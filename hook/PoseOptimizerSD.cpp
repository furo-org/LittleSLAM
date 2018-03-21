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
 * @file PoseOptimizerSD.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PoseOptimizerSD.h"

using namespace std;

////////

// データ対応づけ固定のもと、初期値initPoseを与えてロボット位置の推定値estPoseを求める
double PoseOptimizerSD::optimizePose(Pose2D &initPose, Pose2D &estPose) {
  double th = initPose.th;
  double tx = initPose.tx;
  double ty = initPose.ty;
  double txmin=tx, tymin=ty, thmin=th;         // コスト最小の解
  double evmin = HUGE_VAL;                     // コストの最小値
  double evold = evmin;                        // 1つ前のコスト値。収束判定に使う

  double ev = cfunc->calValue(tx, ty, th);     // コスト計算
  int nn=0;                                    // 繰り返し回数。確認用
  double kk=0.00001;                           // 最急降下法のステップ幅係数
  while (abs(evold-ev) > evthre) {             // 収束判定。1つ前の値との変化が小さいと終了
    nn++;
    evold = ev;

    // 数値計算による偏微分
    double dEtx = (cfunc->calValue(tx+dd, ty, th) - ev)/dd;
    double dEty = (cfunc->calValue(tx, ty+dd, th) - ev)/dd;
    double dEth = (cfunc->calValue(tx, ty, th+da) - ev)/da;

    // 微分係数にkkをかけてステップ幅にする
    double dx = -kk*dEtx;
    double dy = -kk*dEty;
    double dth = -kk*dEth;
    tx += dx;  ty += dy;  th += dth;            // ステップ幅を加えて次の探索位置を決める

    ev = cfunc->calValue(tx, ty, th);           // その位置でコスト計算

    if (ev < evmin) {                           // evがこれまでの最小なら更新
      evmin = ev;
      txmin = tx;  tymin = ty;  thmin = th;
    }

//    printf("nn=%d, ev=%g, evold=%g, abs(evold-ev)=%g\n", nn, ev, evold, abs(evold-ev));         // 確認用
  }

  ++allN;
  if (allN > 0 && evmin < 100) 
    sum += evmin;
//  printf("allN=%d, evmin=%g, avg=%g\n", allN, evmin, (sum/allN));         // 確認用

//  printf("nn=%d, ev=%g\n", nn, ev);         // 確認用

  estPose.setVal(txmin, tymin, thmin);          // 最小値を与える解を保存

  return(evmin);
}
