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
 * @file CostFunctionED.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "CostFunctionED.h"

using namespace std;

// 点間距離によるICPのコスト関数
double CostFunctionED::calValue(double tx, double ty, double th) {
  double a = DEG2RAD(th);
  double error=0;
  int pn=0;
  int nn=0;
  for (size_t i=0; i<curLps.size(); i++) {
    const LPoint2D *clp = curLps[i];             // 現在スキャンの点
    const LPoint2D *rlp = refLps[i];             // clpに対応する参照スキャンの点

    double cx = clp->x;
    double cy = clp->y;
    double x = cos(a)*cx - sin(a)*cy + tx;       // clpを参照スキャンの座標系に変換
    double y = sin(a)*cx + cos(a)*cy + ty;

    double edis = (x - rlp->x)*(x - rlp->x) + (y - rlp->y)*(y - rlp->y);     // 点間距離

    if (edis <= evlimit*evlimit)
      ++pn;                                      // 誤差が小さい点の数

    error += edis;                               // 各点の誤差を累積

    ++nn;
  }

  error = (nn>0)? error/nn : HUGE_VAL;           // 平均をとる。有効点数が0なら、値はHUGE_VAL
  pnrate = 1.0*pn/nn;                            // 誤差が小さい点の比率

//  printf("CostFunctionED: error=%g, pnrate=%g, evlimit=%g\n", error, pnrate, evlimit);     // 確認用

  error *= 100;                                  // 評価値が小さくなりすぎないよう100かける。

  return(error);
}
