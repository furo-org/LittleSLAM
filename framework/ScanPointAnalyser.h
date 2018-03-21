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
 * @file ScanPointAnalyser.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SCAN_ANALYSER_H_
#define SCAN_ANALYSER_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Scan2D.h"


class ScanPointAnalyser
{
private:
  static const double FPDMIN;               // 隣接点との最小距離[m]。これより小さいと誤差が大きくなるので法線計算に使わない。
  static const double FPDMAX;               // 隣接点との最大距離[m]。これより大きいと不連続とみなして法線計算に使わない。
  static const int CRTHRE = 45;             // 法線方向変化の閾値[度]。これより大きいとコーナ点とみなす。
  static const int INVALID = -1;
  double costh;                             // 左右の法線方向の食い違いの閾値

public:
  ScanPointAnalyser() : costh(cos(DEG2RAD(CRTHRE))) {
  }

  ~ScanPointAnalyser() {
  }

//////////
  
  void analysePoints(std::vector<LPoint2D> &lps);
  bool calNormal(int idx, const std::vector<LPoint2D> &lps, int dir, Vector2D &normal);

};

#endif

