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
 * @file SlamBackEnd.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SLAM_BACK_END_H_
#define SLAM_BACK_END_H_

#include <vector>
#include "PointCloudMap.h"
#include "PoseGraph.h"

////////

class SlamBackEnd
{
private:
  std::vector<Pose2D> newPoses;            // ポーズ調整後の姿勢
  PointCloudMap *pcmap;                    // 点群地図
  PoseGraph *pg;                           // ポーズグラフ

public:
  SlamBackEnd() {
  }

  ~SlamBackEnd() {
  }

//////////

  void setPointCloudMap(PointCloudMap *m) {
    pcmap = m;
  }

  void setPoseGraph(PoseGraph *g) {
    pg = g;
  }
  
//////////

  Pose2D adjustPoses();
  void remakeMaps(); 
};

#endif
