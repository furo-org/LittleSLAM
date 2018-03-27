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
 * @file LPoint2D.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef LPOINT2D_H_
#define LPOINT2D_H_

////////////////////////

struct Vector2D
{
  double x,y;
};

////////////////////////

enum ptype {UNKNOWN=0, LINE=1, CORNER=2, ISOLATE=3};    // 点のタイプ：未知、直線、コーナ、孤立

struct LPoint2D
{
  int sid;                 // フレーム番号（スキャン番号）
  double x;                // 位置x
  double y;                // 位置y
  double nx;               // 法線ベクトル
  double ny;               // 法線ベクトル
  double atd;              // 累積走行距離(accumulated travel distance)
  ptype type;              // 点のタイプ

  LPoint2D() : sid(-1), x(0), y(0) {
    init();
  }

  LPoint2D(int id, double _x, double _y): x(_x), y(_y) {
    init();
    sid = id;
  }
 
//////////

  void init() {
    sid = -1;
    atd = 0;
    type = UNKNOWN;
    nx = 0;
    ny = 0;
  }

  void setData(int id, double _x, double _y) {
    init();
    sid = id;
    x = _x;
    y = _y;
  }

  void setXY(double _x, double _y) {
    x = _x;
    y = _y;
  }

  // rangeとangleからxyを求める(右手系)
  void calXY(double range, double angle) {
    double a = DEG2RAD(angle);
    x = range*cos(a);
    y = range*sin(a);
  }

  // rangeとangleからxyを求める(左手系）
  void calXYi(double range, double angle) {
    double a = DEG2RAD(angle);
    x = range*cos(a);
    y = -range*sin(a);
  }

  void setSid(int i) {
    sid = i;
  }

  void setAtd(double t) {
    atd = t;
  }

  void setType(ptype t) {
    type = t;
  }

  void setNormal(double x, double y) {
    nx = x;
    ny = y;
  }

};

#endif
