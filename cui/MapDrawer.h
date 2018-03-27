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
 * @file MapDrawer.h
 * @author Masahiro Tomono
 ****************************************************************************/

#include <stdio.h>
#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"

class MapDrawer
{
private:
  FILE *gp;               // gnuplotへのパイプ
  double xmin;            // 描画範囲[m]
  double xmax;
  double ymin;
  double ymax;
  double aspectR;         // xy比

public:
  MapDrawer() : gp(nullptr), xmin(-10), xmax(10), ymin(-10), ymax(10), aspectR(-1.0) {
  }

  ~MapDrawer() {
    finishGnuplot();
  }
  
/////////

  void initGnuplot() {
#ifdef _WIN32
    gp = _popen("gnuplot", "w");      // パイプオープン.Windows
#elif __linux__
    gp = popen("gnuplot", "w");       // パイプオープン.Linux
#endif
  }

  void finishGnuplot() {
    if (gp != nullptr)
#ifdef _WIN32
      _pclose(gp);
#elif __linux__
      pclose(gp);
#endif
  }

  void setAspectRatio(double a) {
    aspectR = a;
    fprintf(gp, "set size ratio %lf\n", aspectR);
  }

  void setRange(double R) {              // 描画範囲をR四方にする
    xmin = ymin = -R;
    xmax = ymax = R;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xR, double yR) {  // 描画範囲を±xR、±yRにする
    xmin = -xR;
    xmax = xR;
    ymin = -yR; 
    ymax = yR;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

  void setRange(double xm, double xM, double ym, double yM) {  // 描画範囲を全部指定
    xmin = xm;
    xmax = xM;
    ymin = ym; 
    ymax = yM;
    fprintf(gp, "set xrange [%lf:%lf]\n", xmin, xmax);
    fprintf(gp, "set yrange [%lf:%lf]\n", ymin, ymax);
  }

////////

  void drawMapGp(const PointCloudMap &pcmap);
  void drawScanGp(const Scan2D &scan);
  void drawTrajectoryGp(const std::vector<Pose2D> &poses);
  void drawGp(const std::vector<LPoint2D> &lps, const std::vector<Pose2D> &poses, bool flush=true);
};

