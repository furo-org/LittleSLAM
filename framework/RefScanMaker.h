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
 * @file RefScanMaker.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _REF_SCAN_MAKER_H_
#define _REF_SCAN_MAKER_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"

class RefScanMaker
{
protected:
  const PointCloudMap *pcmap;           // 点群地図
  Scan2D refScan;                       // 参照スキャン本体。これを外に提供

public:
  RefScanMaker() : pcmap(nullptr) {
  }

  ~RefScanMaker() {
  }

  void setPointCloudMap(const PointCloudMap *p) {
    pcmap = p;
  }

  virtual const Scan2D *makeRefScan() = 0;

};

#endif
