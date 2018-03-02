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
 * @file RefScanMakerBS.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _REF_SCAN_MAKER_BS_H_
#define _REF_SCAN_MAKER_BS_H_

#include "RefScanMaker.h"

class RefScanMakerBS : public RefScanMaker
{
public:
  RefScanMakerBS() {
  }
  ~RefScanMakerBS() {
  }

  virtual const Scan2D *makeRefScan();

};

#endif
