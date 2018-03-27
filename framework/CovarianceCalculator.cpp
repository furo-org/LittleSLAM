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
 * @file CovarianceCalculator.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "CovarianceCalculator.h"

using namespace std;

////////// ICPによる推定値の共分散 /////////

// ICPによるロボット位置の推定値の共分散covを求める。
// 推定位置pose、現在スキャン点群curLps、参照スキャン点群refLps
double CovarianceCalculator::calIcpCovariance(const Pose2D &pose, std::vector<const LPoint2D*> &curLps, std::vector<const LPoint2D*> &refLps, Eigen::Matrix3d &cov) {
  double tx = pose.tx;
  double ty = pose.ty;
  double th = pose.th;
  double a = DEG2RAD(th);
  vector<double> Jx;                                         // ヤコビ行列のxの列
  vector<double> Jy;                                         // ヤコビ行列のyの列
  vector<double> Jt;                                         // ヤコビ行列のthの列

  for (size_t i=0; i<curLps.size(); i++) {
    const LPoint2D *clp = curLps[i];                         // 現在スキャンの点
    const LPoint2D *rlp = refLps[i];                         // 参照スキャンの点

    if (rlp->type == ISOLATE)                                // 孤立点は除外
      continue;

    double pd0 = calPDistance(clp, rlp, tx, ty, a);         // コスト関数値
    double pdx = calPDistance(clp, rlp, tx+dd, ty, a);      // xを少し変えたコスト関数値
    double pdy = calPDistance(clp, rlp, tx, ty+dd, a);      // yを少し変えたコスト関数値
    double pdt = calPDistance(clp, rlp, tx, ty, a+da);      // thを少し変えたコスト関数値

    Jx.push_back((pdx - pd0)/dd);                            // 偏微分（x成分）
    Jy.push_back((pdy - pd0)/dd);                            // 偏微分（y成分）
    Jt.push_back((pdt - pd0)/da);                            // 偏微分（th成分）
  }

  // ヘッセ行列の近似J^TJの計算
  Eigen::Matrix3d hes = Eigen::Matrix3d::Zero(3,3);          // 近似ヘッセ行列。0で初期化
  for (size_t i=0; i<Jx.size(); i++) {
    hes(0,0) += Jx[i]*Jx[i];
    hes(0,1) += Jx[i]*Jy[i];
    hes(0,2) += Jx[i]*Jt[i];
    hes(1,1) += Jy[i]*Jy[i];
    hes(1,2) += Jy[i]*Jt[i];
    hes(2,2) += Jt[i]*Jt[i];
  }
  // J^TJが対称行列であることを利用
  hes(1,0) = hes(0,1);
  hes(2,0) = hes(0,2);
  hes(2,1) = hes(1,2);

  // 共分散行列は（近似）ヘッセ行列の逆行列
//  cov = hes.inverse();
  cov = MyUtil::svdInverse(hes);                              // SVDを使う方が少しよい

  double vals[2], vec1[2], vec2[2];
  double ratio = calEigen(cov, vals, vec1, vec2);            // 固有値計算して、退化具合を調べる

  // 必要に応じて共分散行列のスケールを調整する
//  double kk = 1;          // 退化で極端にずれる場合
  double kk = 0.1;       // 通常
  cov *= kk;

  return(ratio);
}

// 垂直距離を用いた観測モデルの式
double CovarianceCalculator::calPDistance(const LPoint2D *clp, const LPoint2D *rlp, double tx, double ty, double th) {
  double x = cos(th)*clp->x - sin(th)*clp->y + tx;                     // clpを推定位置で座標変換
  double y = sin(th)*clp->x + cos(th)*clp->y + ty;
  double pdis = (x - rlp->x)*rlp->nx + (y - rlp->y)*rlp->ny;           // 座標変換した点からrlpへの垂直距離

  return(pdis);
}

///////// 運動モデルの計算 /////////

void CovarianceCalculator::calMotionCovarianceSimple(const Pose2D &motion, double dT, Eigen::Matrix3d &cov) {
  double dis = sqrt(motion.tx*motion.tx + motion.ty*motion.ty);   // 移動距離
  double vt = dis/dT;                    // 並進速度[m/s]
  double wt = DEG2RAD(motion.th)/dT;     // 角速度[rad/s]
  double vthre = 0.02;                   // vtの下限値。同期ずれで0になる場合の対処
  double wthre = 0.05;                   // wtの下限値

  if (vt < vthre)
    vt = vthre;
  if (wt < wthre)
    wt = wthre;

  double dx = vt;
  double dy = vt;
  double da = wt;

  Eigen::Matrix3d C1;
  C1.setZero();                          // 対角要素だけ入れる
  C1(0,0) = 0.001*dx*dx;                 // 並進成分x
  C1(1,1) = 0.005*dy*dy;                 // 並進成分y
//  C1(2,2) = 0.005*da*da;                 // 回転成分
  C1(2,2) = 0.05*da*da;                 // 回転成分

  // スケール調整
//  double kk = 100;                     // オドメトリのずれが大きい場合
  double kk = 1;                         // 通常
  cov = kk*C1;

  // 確認用
  printf("calMotionCovarianceSimple\n");
  printf("vt=%g, wt=%g\n", vt, wt);
  double vals[2], vec1[2], vec2[2];
  calEigen(cov, vals, vec1, vec2);
  printf("cov : %g %g %g %g %g %g\n", cov(0,0), cov(0,1), cov(0,2), cov(1,1), cov(1,2), cov(2,2));
}

///////// 運動モデルの計算 /////////

// 1フレーム分の走行による誤差。dTは1フレームの時間。motionはその間の移動量。
void CovarianceCalculator::calMotionCovariance(double th, double dx, double dy, double dth, double dt, Eigen::Matrix3d &cov, bool accum) {
  setAlpha(1, 5);
  double dis = sqrt(dx*dx + dy*dy);   // 走行距離
  double vt = dis/dt;                 // 並進速度[m/s]
  double wt = dth/dt;                 // 角速度[rad/s]
  double vthre = 0.001;               // vtの下限値。タイミングにより0になるのを防ぐ。
  double wthre = 0.01;                // wtの下限値
  if (vt < vthre)
    vt = vthre;
  if (wt < wthre)
    wt = wthre;

 // 累積する場合は、時刻t-1の共分散行列sigmaから、時刻tの共分散行列を計算
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero(3,3);
  if (accum) {
    Eigen::Matrix3d Jxk;
    calJxk(th, vt, dt, Jxk);
    A = Jxk*cov*Jxk.transpose();
  }

  Eigen::Matrix2d Uk;
  calUk(vt, wt, Uk);

  Eigen::Matrix<double, 3, 2> Juk;
  calJuk(th, dt, Juk);
  Eigen::Matrix3d B = Juk*Uk*Juk.transpose();

  cov = A + B;

}

//////////////////////////////////

void CovarianceCalculator::calUk(double vt, double wt, Eigen::Matrix2d &Uk) {
  Uk << a1*vt*vt, 0,
        0, a2*wt*wt;
}

// ロボット姿勢に関するヤコビ行列。vtはロボットの速度、thはロボットの方向(ラジアン)、dtは時間
void CovarianceCalculator::calJxk(double th, double vt, double dt, Eigen::Matrix3d &Jxk) {
  double cs = cos(th);
  double sn = sin(th);
  Jxk << 1, 0, -vt*dt*sn,
         0, 1, vt*dt*cs,
         0, 0, 1;
}

// 
void CovarianceCalculator::calJuk(double th, double dt, Eigen::Matrix<double, 3, 2> &Juk) {
  double cs = cos(th);
  double sn = sin(th);
  Juk << dt*cs, 0,
         dt*sn, 0,
         0, dt;
}

////////////////

// 共分散行列covの並進成分だけを固有値分解し、固有値をvalsに、固有ベクトルをvec1とvec2に入れる。
double CovarianceCalculator::calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2) {
  // 並進部分だけ取り出す
  double cv2[2][2];
  for (int i=0; i<2; i++) 
    for (int j=0; j<2; j++) 
      cv2[i][j] = cov(i,j);

  MyUtil::calEigen2D(cv2, vals, vec1, vec2);        // 固有値分解
  double ratio = vals[0]/vals[1];

  // 確認用
  printf("Eigen: ratio=%g, val1=%g, val2=%g\n", ratio, vals[0], vals[1]);
  printf("Eigen: vec1=(%g, %g), ang=%g\n", vec1[0], vec1[1], RAD2DEG(atan2(vec1[1], vec1[0])));

  return(ratio);
}

//////////////

// 共分散行列の累積。前回位置の共分散行列prevCovに移動量の共分散行列mcovを加えて、現在位置の共分散行列curCovを求める。
void CovarianceCalculator::accumulateCovariance(const Pose2D &curPose, const Pose2D &prevPose, const Eigen::Matrix3d &prevCov, const Eigen::Matrix3d &mcov, Eigen::Matrix3d &curCov) {
  Eigen::Matrix3d J1, J2;
  J1 << 1, 0, -(curPose.ty - prevPose.ty),
        0, 1, curPose.tx - prevPose.tx,
        0, 0, 1;

  double prevCos = cos(DEG2RAD(prevPose.th));
  double prevSin = sin(DEG2RAD(prevPose.th));
  J2 << prevCos, -prevSin, 0,
        prevSin, prevCos, 0,
        0, 0, 1;

  curCov = J1*prevCov*J1.transpose() + J2*mcov*J2.transpose();
}

/////////////

// 共分散行列covをposeの角度分だけ回転させる
void CovarianceCalculator::rotateCovariance(const Pose2D &pose, const Eigen::Matrix3d &cov, Eigen::Matrix3d &icov, bool reverse) {
  double cs = cos(DEG2RAD(pose.th));            // poseの回転成分thによるcos
  double sn = sin(DEG2RAD(pose.th));
  Eigen::Matrix3d J;                            // 回転のヤコビ行列
  J << cs, -sn, 0,
       sn, cs, 0,
       0, 0, 1;

  Eigen::Matrix3d JT = J.transpose();

  if (reverse)
    icov = JT*cov*J;                              // 逆回転変換
  else
    icov = J*cov*JT;                              // 回転変換
}
