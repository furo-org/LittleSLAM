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
 * @file PointCloudMapLP.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef POINT_CLOUD_MAP_LP_H_
#define POINT_CLOUD_MAP_LP_H_

#include <boost/unordered_map.hpp>
#include "PointCloudMap.h"

///////////

// 部分地図
struct Submap
{
  double atdS;                              // 部分地図の始点での累積走行距離
  size_t cntS;                              // 部分地図の最初のスキャン番号
  size_t cntE;                              // 部分地図の最後のスキャン番号

  std::vector<LPoint2D> mps;                // 部分地図内のスキャン点群

  Submap() : atdS(0), cntS(0), cntE(-1) {
  }

  Submap(double a, size_t s) : cntE(-1) {
    atdS = a;
    cntS = s;
  }

  void addPoints(const std::vector<LPoint2D> &lps) {
    for (size_t i=0; i<lps.size(); i++)
      mps.emplace_back(lps[i]);
  }

  std::vector<LPoint2D> subsamplePoints(int nthre);
};

///////////

// 部分地図から構成される点群地図
class PointCloudMapLP : public PointCloudMap
{
public:
  static double atdThre;                    // 部分地図の区切りとなる累積走行距離(atd)[m]
  double atd;                               // 現在の累積走行距離(accumulated travel distance)
  std::vector<Submap> submaps;              // 部分地図

public:
  PointCloudMapLP() {
    Submap submap;
    submaps.emplace_back(submap);           // 最初の部分地図を作っておく
  }

  ~PointCloudMapLP() {
  }

//////////

  Pose2D getLastPose() const {
    return(lastPose);
  }

  void setLastPose(Pose2D &p) {
    lastPose = p;
  }

  std::vector<Submap> &getSubmaps() {
    return(submaps);
  }

/////////////

  virtual void addPose(const Pose2D &p);
  virtual void addPoints(const std::vector<LPoint2D> &lps);
  virtual void makeGlobalMap();
  virtual void makeLocalMap();
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses);

};

#endif
