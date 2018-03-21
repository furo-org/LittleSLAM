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
 * @file SlamLauncher.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SLAM_LAUNCHER_H_
#define SLAM_LAUNCHER_H_

#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#endif

#include "SensorDataReader.h"
#include "PointCloudMap.h"
#include "SlamFrontEnd.h"
#include "SlamBackEnd.h"
#include "MapDrawer.h"
#include "FrameworkCustomizer.h"

/////////////

class SlamLauncher
{
private:
  int startN;                      // 開始スキャン番号
  int drawSkip;                    // 描画間隔
  bool odometryOnly;               // オドメトリによる地図構築か
  Pose2D ipose;                    // オドメトリ地図構築の補助データ。初期位置の角度を0にする

  Pose2D lidarOffset;              // レーザスキャナとロボットの相対位置

  SensorDataReader sreader;        // ファイルからのセンサデータ読み込み
  PointCloudMap *pcmap;            // 点群地図
  SlamFrontEnd sfront;             // SLAMフロントエンド
  MapDrawer mdrawer;               // gnuplotによる描画
  FrameworkCustomizer fcustom;     // フレームワークの改造

public:
  SlamLauncher() : startN(0), drawSkip(10), odometryOnly(false), pcmap(nullptr) {
  }

  ~SlamLauncher() {
  }

///////////

  void setStartN(int n) {
    startN = n;
  }

  void setOdometryOnly(bool p) {
    odometryOnly = p;
  }

///////////

  void run();
  void showScans();
  void mapByOdometry(Scan2D *scan);
  bool setFilename(char *filename);
  void skipData(int num);
  void customizeFramework();
};

#endif
