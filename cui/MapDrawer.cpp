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
 * @file MapDrawer.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "MapDrawer.h"

using namespace std;

////////// Gnuplotによる地図描画 //////////

// 地図と軌跡を描画
void MapDrawer::drawMapGp(const PointCloudMap &pcmap) {
  const vector<LPoint2D> &lps = pcmap.globalMap;         // 地図の点群
  const vector<Pose2D> &poses = pcmap.poses;             // ロボット軌跡
  drawGp(lps, poses);
}

// スキャン1個を描画
void MapDrawer::drawScanGp(const Scan2D &scan) {
  vector<Pose2D> poses;
  Pose2D pose;                   // 原点
  poses.emplace_back(pose);      // drawGpを使うためにvectorに入れる
  drawGp(scan.lps, poses);
}

// ロボット軌跡だけを描画
void MapDrawer::drawTrajectoryGp(const vector<Pose2D> &poses) {
  vector<LPoint2D> lps;          // drawGpを使うためのダミー（空）
  drawGp(lps, poses);
}

//////////

void MapDrawer::drawGp(const vector<LPoint2D> &lps, const vector<Pose2D> &poses, bool flush) {
  printf("drawGp: lps.size=%lu\n", lps.size());     // 点数の確認用

  // gnuplot設定
  fprintf(gp, "set multiplot\n");
//  fprintf(gp, "plot '-' w p pt 7 ps 0.1, '-' with vector\n");
  fprintf(gp, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector\n");

  // 点群の描画
  int step1=1;                  // 点の間引き間隔。描画が重いとき大きくする
  for (size_t i=0; i<lps.size(); i+=step1) {
    const LPoint2D &lp = lps[i];
    fprintf(gp, "%lf %lf\n", lp.x, lp.y);    // 点の描画
  }
  fprintf(gp, "e\n");

  // ロボット軌跡の描画
  int step2=10;                      // ロボット位置の間引き間隔
  for (size_t i=0; i<poses.size(); i+=step2) {
    const Pose2D &pose = poses[i];
    double cx = pose.tx;             // 並進位置
    double cy = pose.ty;
    double cs = pose.Rmat[0][0];     // 回転角によるcos
    double sn = pose.Rmat[1][0];     // 回転角によるsin

    // ロボット座標系の位置と向きを描く
    double dd = 0.4;
    double x1 = cs*dd;              // ロボット座標系のx軸
    double y1 = sn*dd;
    double x2 = -sn*dd;             // ロボット座標系のy軸
    double y2 = cs*dd;
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x1, y1);
    fprintf(gp, "%lf %lf %lf %lf\n", cx, cy, x2, y2);
  }
  fprintf(gp, "e\n");

  if (flush)
    fflush(gp);         // バッファのデータを書き出す。これしないと描画がよくない
}
