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
 * @file DataAssociatorLS.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef DATA_ASSOCIATOR_LS_H_
#define DATA_ASSOCIATOR_LS_H_

#include "DataAssociator.h"

// 線形探索を用いて、現在スキャンと参照スキャン間の点の対応づけを行う
class DataAssociatorLS : public DataAssociator
{
private:
  std::vector<const LPoint2D*> baseLps;              // 参照スキャンの点を格納しておく。作業用

public:
  DataAssociatorLS() {
  }

  ~DataAssociatorLS() {
  }

  // 参照スキャンの点rlpsをポインタにしてbaseLpsに入れる
  virtual void setRefBase(const std::vector<LPoint2D> &rlps) {
    baseLps.clear();
    for (size_t i=0; i<rlps.size(); i++)
      baseLps.push_back(&rlps[i]);                // ポインタにして格納
  }

/////////

  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
