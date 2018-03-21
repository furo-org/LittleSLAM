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
 * @file PoseFuser.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef POSE_FUSER_H_
#define POSE_FUSER_H_

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "DataAssociator.h"
#include "CovarianceCalculator.h"

// センサ融合器。ICPとオドメトリの推定値を融合する。
class PoseFuser
{
public:
  Eigen::Matrix3d ecov;                      // ICPの共分散行列
  Eigen::Matrix3d mcov;                      // オドメトリの共分散行列
  Eigen::Matrix3d totalCov;
  
  DataAssociator *dass;                      // データ対応づけ器
  CovarianceCalculator cvc;                 // 共分散計算器

public:
  PoseFuser() {
  }

  ~PoseFuser() {
  }

/////

  void setDataAssociator(DataAssociator *d) {
    dass = d;
  }

  void setRefScan(const Scan2D *refScan) {
    dass->setRefBase(refScan->lps);
  }

  void setRefLps(const std::vector<LPoint2D> &refLps) {
    dass->setRefBase(refLps);
  }

  // ICPの共分散行列の計算。setRefLpsの後に行うこと。
  double calIcpCovariance(const Pose2D &estMotion, const Scan2D *curScan, Eigen::Matrix3d &cov) {
    dass->findCorrespondence(curScan, estMotion);

    // ICPの共分散。ここで得られるのは、世界座標系での共分散
    double ratio = cvc.calIcpCovariance(estMotion, dass->curLps, dass->refLps, cov);
    return(ratio);
  }

//////////

  double fusePose(Scan2D *curScan, const Pose2D &estPose, const Pose2D &odoMotion, const Pose2D &lastPose, Pose2D &fusedPose, Eigen::Matrix3d &cov);
  void calOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose, Eigen::Matrix3d &mcov);
  double fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,  const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2, Eigen::Vector3d &mu, Eigen::Matrix3d &cv);
  void printMatrix(const Eigen::Matrix3d &mat);

};

#endif
