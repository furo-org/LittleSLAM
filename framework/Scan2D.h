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
 * @file Scan2D.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SCAN2D_H_
#define SCAN2D_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"

//////////

// スキャン
struct Scan2D
{
  static double MAX_SCAN_RANGE;              // スキャン点の距離値上限[m]
  static double MIN_SCAN_RANGE;              // スキャン点の距離値下限[m]

  int sid;                                   // スキャンid
  Pose2D pose;                               // スキャン取得時のオドメトリ値
  std::vector<LPoint2D> lps;                 // スキャン点群

  Scan2D() : sid(0) {
  }

  ~Scan2D() {
  }

///////

  void setSid(int s) {
    sid = s;
  }

  void setLps (const std::vector<LPoint2D> &ps) {
    lps = ps;
  }
  
  void setPose(Pose2D &p) {
    pose = p;
  }

};

#endif
