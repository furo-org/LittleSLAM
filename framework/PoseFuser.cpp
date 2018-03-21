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

////////////// 逐次SLAM用のセンサ融合 ////////////////

// 逐次SLAMでのICPとオドメトリの推定移動量を融合する。dassに参照スキャンを入れておくこと。covに移動量の共分散行列が入る。
double PoseFuser::fusePose(Scan2D *curScan, const Pose2D &estPose, const Pose2D &odoMotion, const Pose2D &lastPose, Pose2D &fusedPose, Eigen::Matrix3d &fusedCov) {
  // ICPの共分散
  dass->findCorrespondence(curScan, estPose);                                      // 推定位置estPoseで現在スキャン点群と参照スキャン点群の対応づけ
  double ratio = cvc.calIcpCovariance(estPose, dass->curLps, dass->refLps, ecov);  // ここで得られるのは、地図座標系での位置の共分散

  // オドメトリの位置と共分散。速度運動モデルを使うと、短期間では共分散が小さすぎるため、簡易版で大きめに計算する
  Pose2D predPose;                                                                 // 予測位置
  Pose2D::calGlobalPose(odoMotion, lastPose, predPose);                            // 直前位置lastPoseに移動量を加えて予測位置を計算
  Eigen::Matrix3d mcovL;
  double dT=0.1;
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);                             // オドメトリで得た移動量の共分散（簡易版）
  CovarianceCalculator::rotateCovariance(estPose, mcovL, mcov);                    // 現在位置estPoseで回転させて、地図座標系での共分散mcovを得る

  // ecov, mcov, covともに、lastPoseを原点とした局所座標系での値
  Eigen::Vector3d mu1(estPose.tx, estPose.ty, DEG2RAD(estPose.th));                // ICPによる推定値
  Eigen::Vector3d mu2(predPose.tx, predPose.ty, DEG2RAD(predPose.th));             // オドメトリによる推定値
  Eigen::Vector3d mu;
  fuse(mu1, ecov, mu2, mcov, mu, fusedCov);                                        // 2つの正規分布の融合

  fusedPose.setVal(mu[0], mu[1], RAD2DEG(mu[2]));                                  // 融合した移動量を格納

  totalCov = fusedCov;

  // 確認用
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
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);                             // オドメトリで得た移動量の共分散（簡易版）
  CovarianceCalculator::rotateCovariance(lastPose, mcovL, mcov);                   // 直前位置lastPoseで回転させて、位置の共分散mcovを得る
}

/////// ガウス分布の融合 ///////

// 2つの正規分布を融合する。muは平均、cvは共分散。
double PoseFuser::fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,  const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2, Eigen::Vector3d &mu, Eigen::Matrix3d &cv) {
  // 共分散行列の融合
  Eigen::Matrix3d IC1 = MyUtil::svdInverse(cv1);
  Eigen::Matrix3d IC2 = MyUtil::svdInverse(cv2);
  Eigen::Matrix3d IC = IC1 + IC2;
  cv = MyUtil::svdInverse(IC);

  // 角度の補正。融合時に連続性を保つため。
  Eigen::Vector3d mu11 = mu1;             // ICPの方向をオドメトリに合せる
  double da = mu2(2) - mu1(2);
  if (da > M_PI) 
    mu11(2) += 2*M_PI;
  else if (da < -M_PI)
    mu11(2) -= 2*M_PI;

  // 平均の融合
  Eigen::Vector3d nu1 = IC1*mu11;
  Eigen::Vector3d nu2 = IC2*mu2;
  Eigen::Vector3d nu3 = nu1 + nu2;
  mu = cv*nu3;

  // 角度の補正。(-pi, pi)に収める
  if (mu(2) > M_PI) 
    mu(2) -= 2*M_PI;
  else if (mu(2) < -M_PI)
    mu(2) += 2*M_PI;

  // 係数部の計算
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
