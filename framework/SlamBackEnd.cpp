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
 * @file SlamBackEnd.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SlamBackEnd.h"
#include "P2oDriver2D.h"

using namespace std;

////////// ポーズ調整 //////////

Pose2D SlamBackEnd::adjustPoses() {
//  pg->printArcs();
//  pg->printNodes();

  newPoses.clear();

  P2oDriver2D p2o;
  p2o.doP2o(*pg, newPoses, 5);                 // 5回くり返す

  return(newPoses.back());
}

/////////////////////////////

void SlamBackEnd::remakeMaps() {
  // PoseGraphの修正
  vector<PoseNode*> &pnodes = pg->nodes;      // ポーズノード
  for (size_t i=0; i<newPoses.size(); i++) {
    Pose2D &npose = newPoses[i];
    PoseNode *pnode = pnodes[i];              // ノードはロボット位置と1:1対応
    pnode->setPose(npose);                    // 各ノードの位置を更新
  }
  printf("newPoses.size=%lu, nodes.size=%lu\n", newPoses.size(), pnodes.size());

  // PointCloudMapの修正
  pcmap->remakeMaps(newPoses);
}
