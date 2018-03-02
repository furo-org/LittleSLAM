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
 * @file P2oDriver2D.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef P2O_DRIVER2D_H_
#define P2O_DRIVER2D_H_

#include <iostream>
#include <fstream>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PoseGraph.h"

//////////

// ポーズグラフ最適化ライブラリkslamを起動する。
class P2oDriver2D
{
public:

  P2oDriver2D() {
  }

  ~P2oDriver2D() {
  }

///////

  void doP2o( PoseGraph &graph, std::vector<Pose2D> &newPoses, int N);

};

#endif
