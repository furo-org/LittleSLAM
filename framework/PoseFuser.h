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

// ƒZƒ“ƒT—Z‡ŠíBICP‚ÆƒIƒhƒƒgƒŠ‚Ì„’è’l‚ð—Z‡‚·‚éB
class PoseFuser
{
public:
  Eigen::Matrix3d ecov;                      // ICP‚Ì‹¤•ªŽUs—ñ
  Eigen::Matrix3d mcov;                      // ƒIƒhƒƒgƒŠ‚Ì‹¤•ªŽUs—ñ
  Eigen::Matrix3d totalCov;
  
  DataAssociator *dass;                      // ƒf[ƒ^‘Î‰ž‚Ã‚¯Ší
  CovarianceCalculator cvc;                 // ‹¤•ªŽUŒvŽZŠí

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

  // ICP‚Ì‹¤•ªŽUs—ñ‚ÌŒvŽZBsetRefLps‚ÌŒã‚És‚¤‚±‚ÆB
  double calIcpCovariance(const Pose2D &estMotion, const Scan2D *curScan, Eigen::Matrix3d &cov) {
    dass->findCorrespondence(curScan, estMotion);

    // ICP‚Ì‹¤•ªŽUB‚±‚±‚Å“¾‚ç‚ê‚é‚Ì‚ÍA¢ŠEÀ•WŒn‚Å‚Ì‹¤•ªŽU
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
