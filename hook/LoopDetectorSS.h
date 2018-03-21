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
 * @file LoopDetectorSS.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef LOOP_DETECTOR_SS_H_
#define LOOP_DETECTOR_SS_H_

#include "LoopDetector.h"
#include "PointCloudMapLP.h"
#include "DataAssociator.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"


////////////

class LoopDetectorSS : public LoopDetector
{
private:
  double radius;                               // 探索半径[m]（現在位置と再訪点の距離閾値）
  double atdthre;                              // 累積走行距離の差の閾値[m]
  double scthre;                               // ICPスコアの閾値

  PointCloudMapLP *pcmap;                      // 点群地図
  CostFunction *cfunc;                         // コスト関数(ICPとは別に使う)
  PoseEstimatorICP *estim;                     // ロボット位置推定器(ICP)
  DataAssociator *dass;                        // データ対応づけ器
  PoseFuser *pfu;                              // センサ融合器

public:
  LoopDetectorSS() : radius(4), atdthre(10), scthre(0.2) {
  }

  ~LoopDetectorSS() {
  }

/////////

  void setPoseEstimator(PoseEstimatorICP *p) {
    estim = p;
  }

  void setPoseFuser(PoseFuser *p) {
    pfu = p;
  }

  void setDataAssociator(DataAssociator *d) {
    dass = d;
  }

  void setCostFunction(CostFunction *f) {
    cfunc = f;
  }

  void setPointCloudMap(PointCloudMapLP *p) {
    pcmap = p;
  }

//////////

  virtual bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);
  void makeLoopArc(LoopInfo &info);
  bool estimateRevisitPose(const Scan2D *curScan, const std::vector<LPoint2D> &refLps, const Pose2D &initPose, Pose2D &revisitPose);

};

#endif
