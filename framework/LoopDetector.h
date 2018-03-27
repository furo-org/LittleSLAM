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
 * @file LoopDetector.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef LOOP_DETECTOR_H_
#define LOOP_DETECTOR_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PoseGraph.h"

///////

// ループアーク設定情報
struct LoopInfo
{
  bool arcked;                   // すでにポーズアークを張ったか
  int curId;                     // 現在キーフレームid（スキャン）
  int refId;                     // 参照キーフレームid（スキャン，または，LocalGridMap2D）
  Pose2D pose;                   // 現在キーフレームが参照キーフレームにマッチするグローバル姿勢（Gridベースの場合は逆）
  double score;                  // ICPマッチングスコア
  Eigen::Matrix3d cov;           // 共分散

  LoopInfo() : arcked(false), curId(-1), refId(-1), score(-1) {
  }

  ~LoopInfo() {
  }
  
  void setArcked(bool t) {
    arcked = t;
  }
};

//////////////

// デバッグ用データ
struct LoopMatch
{
  Scan2D curScan;
  Scan2D refScan;
  LoopInfo info;

  LoopMatch() {
  }

  LoopMatch(Scan2D &cs, Scan2D &rs, LoopInfo &i) {
    curScan = cs;
    refScan = rs;
    info = i;
  }
};

////////////

class LoopDetector
{
protected:  
  PoseGraph *pg;                               // ポーズグラフ
  std::vector<LoopMatch> loopMatches;          // デバッグ用

public:
  LoopDetector() {
  }

  ~LoopDetector() {
  }

////////

  // デバッグ用
  std::vector<LoopMatch> &getLoopMatches() {
    return(loopMatches);
  }

  void setPoseGraph(PoseGraph *p) {
    pg = p;
  }

///////

  virtual bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);

};

#endif
