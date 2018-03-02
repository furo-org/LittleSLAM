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
 * @file PoseFuser.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "PoseFuser.h"

using namespace std;

////////////// ’€ŽŸSLAM—p‚ÌƒZƒ“ƒT—Z‡ ////////////////

// ’€ŽŸSLAM‚Å‚ÌICP‚ÆƒIƒhƒƒgƒŠ‚Ì„’èˆÚ“®—Ê‚ð—Z‡‚·‚éBdass‚ÉŽQÆƒXƒLƒƒƒ“‚ð“ü‚ê‚Ä‚¨‚­‚±‚ÆBcov‚ÉˆÚ“®—Ê‚Ì‹¤•ªŽUs—ñ‚ª“ü‚éB
double PoseFuser::fusePose(Scan2D *curScan, const Pose2D &estPose, const Pose2D &odoMotion, const Pose2D &lastPose, Pose2D &fusedPose, Eigen::Matrix3d &fusedCov) {
  // ICP‚Ì‹¤•ªŽU
  dass->findCorrespondence(curScan, estPose);                                      // „’èˆÊ’uestPose‚ÅŒ»ÝƒXƒLƒƒƒ““_ŒQ‚ÆŽQÆƒXƒLƒƒƒ““_ŒQ‚Ì‘Î‰ž‚Ã‚¯
  double ratio = cvc.calIcpCovariance(estPose, dass->curLps, dass->refLps, ecov);  // ‚±‚±‚Å“¾‚ç‚ê‚é‚Ì‚ÍA’n}À•WŒn‚Å‚ÌˆÊ’u‚Ì‹¤•ªŽU

  // ƒIƒhƒƒgƒŠ‚ÌˆÊ’u‚Æ‹¤•ªŽUB‘¬“x‰^“®ƒ‚ƒfƒ‹‚ðŽg‚¤‚ÆA’ZŠúŠÔ‚Å‚Í‹¤•ªŽU‚ª¬‚³‚·‚¬‚é‚½‚ßAŠÈˆÕ”Å‚Å‘å‚«‚ß‚ÉŒvŽZ‚·‚é
  Pose2D predPose;                                                                 // —\‘ªˆÊ’u
  Pose2D::calGlobalPose(odoMotion, lastPose, predPose);                            // ’¼‘OˆÊ’ulastPose‚ÉˆÚ“®—Ê‚ð‰Á‚¦‚Ä—\‘ªˆÊ’u‚ðŒvŽZ
  Eigen::Matrix3d mcovL;
  double dT=0.1;
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);                             // ƒIƒhƒƒgƒŠ‚Å“¾‚½ˆÚ“®—Ê‚Ì‹¤•ªŽUiŠÈˆÕ”Åj
  CovarianceCalculator::rotateCovariance(estPose, mcovL, mcov);                    // Œ»ÝˆÊ’uestPose‚Å‰ñ“]‚³‚¹‚ÄA’n}À•WŒn‚Å‚Ì‹¤•ªŽUmcov‚ð“¾‚é

  // ecov, mcov, cov‚Æ‚à‚ÉAlastPose‚ðŒ´“_‚Æ‚µ‚½‹ÇŠÀ•WŒn‚Å‚Ì’l
  Eigen::Vector3d mu1(estPose.tx, estPose.ty, DEG2RAD(estPose.th));                // ICP‚É‚æ‚é„’è’l
  Eigen::Vector3d mu2(predPose.tx, predPose.ty, DEG2RAD(predPose.th));             // ƒIƒhƒƒgƒŠ‚É‚æ‚é„’è’l
  Eigen::Vector3d mu;
  fuse(mu1, ecov, mu2, mcov, mu, fusedCov);                                        // 2‚Â‚Ì³‹K•ª•z‚Ì—Z‡

  fusedPose.setVal(mu[0], mu[1], RAD2DEG(mu[2]));                                  // —Z‡‚µ‚½ˆÚ“®—Ê‚ðŠi”[

  totalCov = fusedCov;

  // Šm”F—p
  printf("fusePose\n");
  double vals[2], vec1[2], vec2[2];
  printf("ecov: det=%g, ", ecov.determinant());
  cvc.calEigen(ecov, vals, vec1, vec2);
  printf("mcov: det=%g, ", mcov.determinant());
  cvc.calEigen(mcov, vals, vec1, vec2);
  printf("fusedCov: det=%g, ", fusedCov.determinant());
  cvc.calEigen(fusedCov, vals, vec1, vec2);

  printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty, predPose.th);
  printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
  printf("fusedPose: tx=%g, ty=%g, th=%g\n", fusedPose.tx, fusedPose.ty, fusedPose.th);

  return(ratio);
}

void PoseFuser::calOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose, Eigen::Matrix3d &mcov) {
  Eigen::Matrix3d mcovL;
  double dT=0.1;
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);                             // ƒIƒhƒƒgƒŠ‚Å“¾‚½ˆÚ“®—Ê‚Ì‹¤•ªŽUiŠÈˆÕ”Åj
  CovarianceCalculator::rotateCovariance(lastPose, mcovL, mcov);                   // ’¼‘OˆÊ’ulastPose‚Å‰ñ“]‚³‚¹‚ÄAˆÊ’u‚Ì‹¤•ªŽUmcov‚ð“¾‚é
}

/////// ƒKƒEƒX•ª•z‚Ì—Z‡ ///////

// 2‚Â‚Ì³‹K•ª•z‚ð—Z‡‚·‚éBmu‚Í•½‹ÏAcv‚Í‹¤•ªŽUB
double PoseFuser::fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,  const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2, Eigen::Vector3d &mu, Eigen::Matrix3d &cv) {
  // ‹¤•ªŽUs—ñ‚Ì—Z‡
  Eigen::Matrix3d IC1 = MyUtil::svdInverse(cv1);
  Eigen::Matrix3d IC2 = MyUtil::svdInverse(cv2);
  Eigen::Matrix3d IC = IC1 + IC2;
  cv = MyUtil::svdInverse(IC);

  // Šp“x‚Ì•â³B—Z‡Žž‚É˜A‘±«‚ð•Û‚Â‚½‚ßB
  Eigen::Vector3d mu11 = mu1;             // ICP‚Ì•ûŒü‚ðƒIƒhƒƒgƒŠ‚É‡‚¹‚é
  double da = mu2(2) - mu1(2);
  if (da > M_PI) 
    mu11(2) += 2*M_PI;
  else if (da < -M_PI)
    mu11(2) -= 2*M_PI;

  // •½‹Ï‚Ì—Z‡
  Eigen::Vector3d nu1 = IC1*mu11;
  Eigen::Vector3d nu2 = IC2*mu2;
  Eigen::Vector3d nu3 = nu1 + nu2;
  mu = cv*nu3;

  // Šp“x‚Ì•â³B(-pi, pi)‚ÉŽû‚ß‚é
  if (mu(2) > M_PI) 
    mu(2) -= 2*M_PI;
  else if (mu(2) < -M_PI)
    mu(2) += 2*M_PI;

  // ŒW”•”‚ÌŒvŽZ
  Eigen::Vector3d W1 = IC1*mu11;
  Eigen::Vector3d W2 = IC2*mu2;
  Eigen::Vector3d W = IC*mu;
  double A1 = mu1.dot(W1);
  double A2 = mu2.dot(W2);
  double A = mu.dot(W);
  double K = A1+A2-A;

/*
  printf("cv1: det=%g\n", cv1.determinant());
  printMatrix(cv1);
  printf("cv2: det=%g\n", cv2.determinant());
  printMatrix(cv2);
  printf("cv: det=%g\n", cv.determinant());
  printMatrix(cv);
*/

  return(K);
}

void PoseFuser::printMatrix(const Eigen::Matrix3d &mat) {
  for (int i=0; i<3; i++) 
    printf("%g %g %g\n", mat(i,0), mat(i,1), mat(i,2));
}
