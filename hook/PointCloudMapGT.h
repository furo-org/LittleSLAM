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
 * @file PointCloudMapGT.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef POINT_CLOUD_MAP_GT_H_
#define POINT_CLOUD_MAP_GT_H_

#include <boost/unordered_map.hpp>
#include "PointCloudMap.h"
#include "NNGridTable.h"

///////////

// 格子テーブルを用いた点群地図
class PointCloudMapGT : public PointCloudMap
{
public:
  std::vector<LPoint2D> allLps;             // 全スキャン点群
  NNGridTable nntab;                        // 格子テーブル

public:
  PointCloudMapGT() {
    allLps.reserve(MAX_POINT_NUM);          // 最初に確保
  }

  ~PointCloudMapGT() {
  }

/////////////

  virtual void addPose(const Pose2D &p);
  virtual void addPoints(const std::vector<LPoint2D> &lps);
  virtual void makeGlobalMap();
  virtual void makeLocalMap();
  void subsamplePoints(std::vector<LPoint2D> &sps);
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif
