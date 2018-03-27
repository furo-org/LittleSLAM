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
 * @file RefScanMakerLM.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "RefScanMakerLM.h"

using namespace std;


const Scan2D *RefScanMakerLM::makeRefScan() {
  vector<LPoint2D> &refLps = refScan.lps;         // 参照スキャンの点群のコンテナ
  refLps.clear();

  const vector<LPoint2D> &localMap = pcmap->localMap;  // 点群地図の局所地図
  for (size_t i=0; i<localMap.size(); i++) {
    const LPoint2D &rp = localMap[i];
    refLps.emplace_back(rp);                           // 単にコピー
  }

  return(&refScan);
}
