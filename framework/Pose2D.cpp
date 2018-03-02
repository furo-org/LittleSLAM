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
 * @file Pose2D.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "Pose2D.h"

// グローバル座標系での点pを、自分（Pose2D）の局所座標系に変換
LPoint2D Pose2D::relativePoint(const LPoint2D &p) const {
  double dx = p.x - tx;
  double dy = p.y - ty;
  double x = dx*Rmat[0][0] + dy*Rmat[1][0];  // 回転の逆行列
  double y = dx*Rmat[0][1] + dy*Rmat[1][1];
  return LPoint2D(p.sid, x, y);
}

////////

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換
LPoint2D Pose2D::globalPoint(const LPoint2D &p) const {
  double x = Rmat[0][0]*p.x + Rmat[0][1]*p.y + tx;
  double y = Rmat[1][0]*p.x + Rmat[1][1]*p.y + ty;
  return LPoint2D(p.sid, x, y);
}

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換してpoに入れる
void Pose2D::globalPoint(const LPoint2D &pi, LPoint2D &po) const {
  po.x = Rmat[0][0]*pi.x + Rmat[0][1]*pi.y + tx;
  po.y = Rmat[1][0]*pi.x + Rmat[1][1]*pi.y + ty;
}

///////

// 基準座標系bposeから見た現座標系nposeの相対位置relPoseを求める（Inverse compounding operator）
void Pose2D::calRelativePose(const Pose2D &npose, const Pose2D &bpose, Pose2D &relPose) {
  const double (*R0)[2] = bpose.Rmat;           // 基準座標系
  const double (*R1)[2] = npose.Rmat;           // 現座標系
  double (*R2)[2] = relPose.Rmat;               // 相対位置

  // 並進
  double dx = npose.tx - bpose.tx;
  double dy = npose.ty - bpose.ty;
  relPose.tx = R0[0][0]*dx + R0[1][0]*dy;
  relPose.ty = R0[0][1]*dx + R0[1][1]*dy;

  // 回転
  double th = npose.th - bpose.th;
  if (th < -180)
    th += 360;
  else if (th >= 180)
    th -= 360;
  relPose.th = th;

  relPose.calRmat();
}

// 基準座標系bposeから相対位置relPoseだけ進んだ、座標系nposeを求める（Compounding operator）
void Pose2D::calGlobalPose(const Pose2D &relPose, const Pose2D &bpose, Pose2D &npose) {
  const double (*R0)[2] = bpose.Rmat;           // 基準座標系
  const double (*R1)[2] = relPose.Rmat;         // 相対位置
  double (*R2)[2] = npose.Rmat;                 // 新座標系

  // 並進
  double tx = relPose.tx;
  double ty = relPose.ty;
  npose.tx =  R0[0][0]*tx + R0[0][1]*ty + bpose.tx;
  npose.ty =  R0[1][0]*tx + R0[1][1]*ty + bpose.ty;

  // 角度
  double th = bpose.th + relPose.th;
  if (th < -180)
    th += 360;
  else if (th >= 180)
    th -= 360;
  npose.th = th;

  npose.calRmat();
}
