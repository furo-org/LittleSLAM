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
 * @file PointCloudMapBS.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PointCloudMapBS.h"

using namespace std;

///////////

// ロボット位置の追加
void PointCloudMapBS::addPose(const Pose2D &p) {
  poses.emplace_back(p);
}

// スキャン点群の追加
void PointCloudMapBS::addPoints(const vector<LPoint2D> &lps) {
  int skip=5;                                       // さすがに重いので、1/5に間引く
//  int skip=10;                                       // さすがに重いので、1/10に間引く
  for (size_t i=0; i<lps.size(); i+=skip) {
    globalMap.emplace_back(lps[i]);                 // 全体地図に追加するだけ
  }
}

// 全体地図生成。すでにできているので何もしない
void PointCloudMapBS::makeGlobalMap(){
  printf("globalMap.size=%lu\n", globalMap.size());   // 確認用
}

// 局所地図生成。ダミー
void PointCloudMapBS::makeLocalMap(){
//  localMap = globalMap;
}

////////

// ダミー
void PointCloudMapBS::remakeMaps(const vector<Pose2D> &newPoses) {
}
