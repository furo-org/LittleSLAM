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
 * @file PointCloudMapGT.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PointCloudMapGT.h"

using namespace std;

///////////

// ロボット位置の追加
void PointCloudMapGT::addPose(const Pose2D &p) {
  poses.emplace_back(p);
}

// 格子テーブルの各セルの代表点を求めてspsに格納する
void PointCloudMapGT::subsamplePoints(vector<LPoint2D> &sps) {
  nntab.clear();                            // 格子テーブルの初期化
  for (size_t i=0; i<allLps.size(); i++) 
    nntab.addPoint(&(allLps[i]));           // 全点を格子テーブルに登録

  nntab.makeCellPoints(nthre, sps);         // nthre点以上あるセルから代表点を得る

  printf("allLps.size=%lu, sps.size=%lu\n", allLps.size(), sps.size());  // 確認用
}

/////////

// スキャン点群を追加
void PointCloudMapGT::addPoints(const vector<LPoint2D> &lps) {
  for (size_t i=0; i<lps.size(); i++)
    allLps.emplace_back(lps[i]);
}

// 全体地図の生成
void PointCloudMapGT::makeGlobalMap(){
  globalMap.clear();
  subsamplePoints(globalMap);         // 格子テーブルの各セルの代表点から全体地図を作る

  printf("GT: globalMap.size=%lu\n", globalMap.size());    // 確認用
}

// 局所地図の生成。全体地図をそのまま使う
void PointCloudMapGT::makeLocalMap(){
  localMap = globalMap;
  printf("GT: localMap.size=%lu\n", localMap.size());
}

////////

// ダミー
void PointCloudMapGT::remakeMaps(const vector<Pose2D> &newPoses) {
}
