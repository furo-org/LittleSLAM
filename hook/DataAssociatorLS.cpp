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
 * @file DataAssociatorLS.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include <boost/timer.hpp>
#include "DataAssociatorLS.h"

using namespace std;

// 現在スキャンcurScanの各スキャン点に対応する点をbaseLpsから見つける
double DataAssociatorLS::findCorrespondence(const Scan2D *curScan, const Pose2D &predPose) {
  boost::timer tim;                                 // 処理時間測定用

  double dthre = 0.2;                               // これより遠い点は除外する[m]
  curLps.clear();                                   // 対応づけ現在スキャン点群を空にする
  refLps.clear();                                   // 対応づけ参照スキャン点群を空にする
  for (size_t i=0; i<curScan->lps.size(); i++) {
    const LPoint2D *clp = &(curScan->lps[i]);       // 現在スキャンの点。ポインタで。

    // スキャン点lpをpredPoseで座標変換した位置に最も近い点を見つける
    LPoint2D glp;                                   // clpの予測位置
    predPose.globalPoint(*clp, glp);                // predPoseで座標変換

    double dmin = HUGE_VAL;                         // 距離最小値
    const LPoint2D *rlpmin = nullptr;               // 最も近い点
    for (size_t j=0; j<baseLps.size(); j++) {
      const LPoint2D *rlp = baseLps[j];             // 参照スキャン点
      double d = (glp.x - rlp->x)*(glp.x - rlp->x) + (glp.y - rlp->y)*(glp.y - rlp->y);
      if (d <= dthre*dthre && d < dmin) {           // dthre内で距離が最小となる点を保存
        dmin = d;
        rlpmin = rlp;
      }
    }
    if (rlpmin != nullptr) {                        // 最近傍点があれば登録
      curLps.push_back(clp);
      refLps.push_back(rlpmin);
    }
  }
  
  double ratio = (1.0*curLps.size())/curScan->lps.size();         // 対応がとれた点の比率
//  printf("ratio=%g, clps.size=%lu\n", ratio, curScan->lps.size());

//  double t1 = 1000*tim.elapsed();                               // 処理時間
//  printf("Elapsed time: dassLS=%g\n", t1);

  return(ratio);
}
