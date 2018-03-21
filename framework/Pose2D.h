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
 * @file Pose2D.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef _POSE2D_H_
#define _POSE2D_H_

#include "MyUtil.h"
#include "LPoint2D.h"

/////////

struct Pose2D
{
  double tx;                           // 並進x
  double ty;                           // 並進y
  double th;                           // 回転角(度)
  double Rmat[2][2];                   // 姿勢の回転行列

  Pose2D() : tx(0), ty(0), th(0) {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        Rmat[i][j] = (i==j)? 1.0:0.0;
      }
    }
  }

  Pose2D(double tx, double ty, double th) {
    this->tx = tx;
    this->ty = ty;
    this->th = th;
    calRmat();
  }

  Pose2D(double mat[2][2], double tx, double ty, double th) {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        Rmat[i][j] = mat[i][j];
      }
    }
    this->tx = tx;
    this->ty = ty;
    this->th = th;
  }

/////////////////

  void reset() {
    tx = ty = th = 0;
    calRmat();
  }

  void setVal(double x, double y, double a) {
    tx = x;
    ty = y;
    th = a;
    calRmat();
  }

  void calRmat() {
    double a = DEG2RAD(th);
    Rmat[0][0] = Rmat[1][1] = cos(a);
    Rmat[1][0] = sin(a);
    Rmat[0][1] = -Rmat[1][0];
  }

  void setTranslation(double tx, double ty) {
    this->tx = tx;
    this->ty = ty;
  }

  void setAngle(double th) {
    this->th = th;
  }

///////////

  LPoint2D relativePoint(const LPoint2D &p) const;
  LPoint2D globalPoint(const LPoint2D &p) const;
  void globalPoint(const LPoint2D &pi, LPoint2D &po) const;

  static void calRelativePose(const Pose2D &npose, const Pose2D &bpose, Pose2D &relPose);
  static void calGlobalPose(const Pose2D &relPose, const Pose2D &bpose, Pose2D &npose);
  
};

///////

struct PoseCov
{
  Pose2D pose;
  Eigen::Matrix3d cov;

  PoseCov() {
  }

  PoseCov(Pose2D &p, Eigen::Matrix3d &c) {
    pose = p;
    cov = c;
  }
};

#endif