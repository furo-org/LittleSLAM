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
 * @file DataAssociatorGT.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef DATA_ASSOCIATOR_GT_H_
#define DATA_ASSOCIATOR_GT_H_

#include "DataAssociator.h"
#include "NNGridTable.h"

// 格子テーブルを用いて、現在スキャンと参照スキャン間の点の対応づけを行う
class DataAssociatorGT : public DataAssociator
{
private:
  NNGridTable nntab;                        // 格子テーブル
  
public:
  DataAssociatorGT() {
  }

  ~DataAssociatorGT() {
  }
  
  // 参照スキャンの点rlpsをポインタにしてnntabに入れる
  virtual void setRefBase(const std::vector<LPoint2D> &rlps) {
    nntab.clear();
    for (size_t i=0; i<rlps.size(); i++) 
      nntab.addPoint(&rlps[i]);              // ポインタにして格納
  }

/////////

  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
