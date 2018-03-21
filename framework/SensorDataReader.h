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
 * @file SensorDataReader.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SENSOR_DATA_READER_H_
#define SENSOR_DATA_READER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"

/////////

class SensorDataReader
{
private:
  int angleOffset;                      // レーザスキャナとロボットの向きのオフセット
  std::ifstream inFile;                 // データファイル

public:
  SensorDataReader() : angleOffset(180) {
  }

  ~SensorDataReader() {
  }

////////

  bool openScanFile(const char *filepath) {
    inFile.open(filepath);
    if (!inFile.is_open()) {
      std::cerr << "Error: cannot open file " << filepath << std::endl;
      return(false);
    }

    return(true);
  }

  void closeScanFile() {
    inFile.close();
  }

  void setAngleOffset(int o) {
     angleOffset = o;
  }

//////////

  bool loadScan(size_t cnt, Scan2D &scan);
  bool loadLaserScan(size_t cnt, Scan2D &scan);
};

#endif
