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
 * @file PoseOptimizerSD.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _POSE_OPTIMIZER_SD_H_
#define _POSE_OPTIMIZER_SD_H_

#include "PoseOptimizer.h"

// 最急降下法でコスト関数を最小化する
class PoseOptimizerSD : public PoseOptimizer
{
public:
  PoseOptimizerSD() {
  }

  ~PoseOptimizerSD() {
  }

/////

  virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);
};

#endif 
