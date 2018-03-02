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
 * @file ScanMatcher2D.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "ScanMatcher2D.h"

using namespace std;

/////////

// スキャンマッチングの実行
bool ScanMatcher2D::matchScan(Scan2D &curScan) {
  ++cnt;

  printf("----- ScanMatcher2D: cnt=%d start -----\n", cnt);

  // spresが設定されていれば、スキャン点間隔を均一化する
  if (spres != nullptr)
    spres->resamplePoints(&curScan);

  // spanaが設定されていれば、スキャン点の法線を計算する
  if (spana != nullptr)
    spana->analysePoints(curScan.lps);

  // 最初のスキャンは単に地図に入れるだけ
  if (cnt == 0) {
    growMap(curScan, initPose);
    prevScan = curScan;                      // 直前スキャンの設定
    return(true);
  }

  // Scanに入っているオドメトリ値を用いて移動量を計算する
  Pose2D odoMotion;                                                   // オドメトリに基づく移動量
  Pose2D::calRelativePose(curScan.pose, prevScan.pose, odoMotion);    // 前スキャンとの相対位置が移動量

  Pose2D lastPose = pcmap->getLastPose();                        // 直前位置
  Pose2D predPose;                                               // オドメトリによる予測位置
  Pose2D::calGlobalPose(odoMotion, lastPose, predPose);          // 直前位置に移動量を加えて予測位置を得る

  const Scan2D *refScan = rsm->makeRefScan();                    // 参照スキャンの生成
  estim->setScanPair(&curScan, refScan);                         // ICPにスキャンを設定
  printf("curScan.size=%lu, refScan.size=%lu\n", curScan.lps.size(), refScan->lps.size());

  Pose2D estPose;                                                // ICPによる推定位置
  double score = estim->estimatePose(predPose, estPose);         // 予測位置を初期値にしてICPを実行
  size_t usedNum = estim->getUsedNum();

  bool successful;                                               // スキャンマッチングに成功したかどうか
  if (score <= scthre && usedNum >= nthre)                       // スコアが閾値より小さければ成功とする
    successful = true;
  else 
    successful = false;
  printf("score=%g, usedNum=%lu, successful=%d\n", score, usedNum, successful);

  if (dgcheck) {                         // 退化の対処をする場合
    if (successful) {
      Pose2D fusedPose;                       // 融合結果
      Eigen::Matrix3d fusedCov;               // センサ融合後の共分散
      pfu->setRefScan(refScan);
      // センサ融合器pfuで、ICP結果とオドメトリ値を融合する
      double ratio = pfu->fusePose(&curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov);
      estPose = fusedPose;
      cov = fusedCov;
      printf("ratio=%g. Pose fused.\n", ratio);     // ratioは退化度。確認用

      // 共分散を累積する
      Eigen::Matrix3d covL;               // 移動量の共分散
      CovarianceCalculator::rotateCovariance(lastPose, fusedCov, covL, true);          // 移動量の共分散に変換
      Eigen::Matrix3d tcov;                // 累積後の共分散
      CovarianceCalculator::accumulateCovariance(lastPose, estPose, totalCov, covL, tcov);
      totalCov = tcov;
    }
    else {                                   // ICP成功でなければ、オドメトリによる予測位置を使う
      estPose = predPose;
      pfu->calOdometryCovariance(odoMotion, lastPose, cov);       // covはオドメトリ共分散だけ
    }
  }
  else {
    if (!successful)
      estPose = predPose;
  }

  growMap(curScan, estPose);               // 地図にスキャン点群を追加
  prevScan = curScan;                      // 直前スキャンの設定

  // 確認用
//  printf("lastPose: tx=%g, ty=%g, th=%g\n", lastPose.tx, lastPose.ty, lastPose.th);
  printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty, predPose.th);     // 確認用
  printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
  printf("cov: %g, %g, %g, %g\n", totalCov(0,0), totalCov(0,1), totalCov(1,0), totalCov(1,1));
  printf("mcov: %g, %g, %g, %g\n", pfu->mcov(0,0), pfu->mcov(0,1), pfu->mcov(1,0), pfu->mcov(1,1));
  printf("ecov: %g, %g, %g, %g\n", pfu->ecov(0,0), pfu->ecov(0,1), pfu->ecov(1,0), pfu->ecov(1,1));

  // 共分散の保存（確認用）
//  PoseCov pcov(estPose, cov);
//  PoseCov pcov(estPose, totalCov);
//  PoseCov pcov(estPose, pfu->mcov);
  PoseCov pcov(estPose, pfu->ecov);
  poseCovs.emplace_back(pcov);

  // 累積走行距離の計算（確認用）
  Pose2D estMotion;                                                    // 推定移動量
  Pose2D::calRelativePose(estPose, lastPose, estMotion);
  atd += sqrt(estMotion.tx*estMotion.tx + estMotion.ty*estMotion.ty); 
  printf("atd=%g\n", atd);

  return(successful);
}

////////////////////

// 現在スキャンを追加して、地図を成長させる
void ScanMatcher2D::growMap(const Scan2D &scan, const Pose2D &pose) {
  const vector<LPoint2D> &lps = scan.lps;                // スキャン点群(ロボット座標系)
  const double (*R)[2] = pose.Rmat;                      // 推定したロボット位置
  double tx = pose.tx;
  double ty = pose.ty;

  vector<LPoint2D> scanG;                                // 地図座標系での点群
  for(size_t i=0; i<lps.size(); i++) {
    const LPoint2D &lp = lps[i];
    if (lp.type == ISOLATE)                              // 孤立点（法線なし）は除外
      continue;
    double x = R[0][0]*lp.x + R[0][1]*lp.y + tx;         // 地図座標系に変換
    double y = R[1][0]*lp.x + R[1][1]*lp.y + ty;
    double nx = R[0][0]*lp.nx + R[0][1]*lp.ny;           // 法線ベクトルも変換
    double ny = R[1][0]*lp.nx + R[1][1]*lp.ny;

    LPoint2D mlp(cnt, x, y);                             // 新規に点を生成
    mlp.setNormal(nx, ny);
    mlp.setType(lp.type);
    scanG.emplace_back(mlp);                             // mlpはvector内にコピーされる
  }

  // 点群地図pcmapに登録
  pcmap->addPose(pose);
  pcmap->addPoints(scanG);
  pcmap->setLastPose(pose);
  pcmap->setLastScan(scan);          // 参照スキャン用に保存
  pcmap->makeLocalMap();             // 局所地図を生成
  
  printf("ScanMatcher: estPose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);    // 確認用
}
