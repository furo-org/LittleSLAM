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
 * @file PoseOptimizerSL.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _POSE_OPTIMIZER_SL_H_
#define _POSE_OPTIMIZER_SL_H_

#include "PoseOptimizer.h"

// íºê¸íTçıÇ¬Ç´ÇÃç≈ã}ç~â∫ñ@Ç≈ÉRÉXÉgä÷êîÇç≈è¨âªÇ∑ÇÈ
class PoseOptimizerSL : public PoseOptimizer
{
public:
  PoseOptimizerSL() {
  }

  ~PoseOptimizerSL() {
  }

/////

  virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);
  double search(double ev0, Pose2D &pose, Pose2D &dp);
  double objFunc(double tt, Pose2D &pose, Pose2D &dp);
};

#endif 
