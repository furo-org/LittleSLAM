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
 * @file PointCloudMapBS.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _POINT_CLOUD_MAP_BS_H_
#define _POINT_CLOUD_MAP_BS_H_

#include "PointCloudMap.h"

// スキャン点をすべて保存する点群地図
class PointCloudMapBS : public PointCloudMap
{
public:
  PointCloudMapBS() {
  }

  ~PointCloudMapBS() {
  }

////////

  virtual void addPose(const Pose2D &p);
  virtual void addPoints(const std::vector<LPoint2D> &lps);
  virtual void makeGlobalMap();
  virtual void makeLocalMap();
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif
