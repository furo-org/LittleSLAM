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
 * @file SlamLauncher.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include <boost/algorithm/string/predicate.hpp>
#include <boost/timer.hpp>
#include "SlamLauncher.h"
#include "ScanPointResampler.h"

using namespace std;                       // C++標準ライブラリの名前空間を使う

//////////

void SlamLauncher::run() {
  mdrawer.initGnuplot();                   // gnuplot初期化
  mdrawer.setAspectRatio(-0.9);            // x軸とy軸の比（負にすると中身が一定）
  
  size_t cnt = 0;                          // 処理の論理時刻
  if (startN > 0)
    skipData(startN);                      // startNまでデータを読み飛ばす

  double totalTime=0, totalTimeDraw=0, totalTimeRead=0;
  Scan2D scan;
  bool eof = sreader.loadScan(cnt, scan);  // ファイルからスキャンを1個読み込む
  boost::timer tim;
  while(!eof) {
    if (odometryOnly) {                      // オドメトリによる地図構築（SLAMより優先）
      if (cnt == 0) {
        ipose = scan.pose;
        ipose.calRmat();
      }
      mapByOdometry(&scan);
    }
    else 
      sfront.process(scan);                // SLAMによる地図構築

    double t1 = 1000*tim.elapsed();

    if (cnt%drawSkip == 0) {               // drawSkipおきに結果を描画
      mdrawer.drawMapGp(*pcmap);
    }
    double t2 = 1000*tim.elapsed();

    ++cnt;                                 // 論理時刻更新
    eof = sreader.loadScan(cnt, scan);     // 次のスキャンを読み込む

    double t3 = 1000*tim.elapsed();
    totalTime = t3;                        // 全体処理時間
    totalTimeDraw += (t2-t1);              // 描画時間の合計
    totalTimeRead += (t3-t2);              // ロード時間の合計

    printf("---- SlamLauncher: cnt=%lu ends ----\n", cnt);
  }
  sreader.closeScanFile();

  printf("Elapsed time: mapping=%g, drawing=%g, reading=%g\n", (totalTime-totalTimeDraw-totalTimeRead), totalTimeDraw, totalTimeRead);
  printf("SlamLauncher finished.\n");

  // 処理終了後も描画画面を残すためにsleepで無限ループにする。ctrl-Cで終了。
  while(true) {
#ifdef _WIN32
    Sleep(1000);                            // WindowsではSleep
#elif __linux__
    usleep(1000000);                        // Linuxではusleep
#endif
  }
}

// 開始からnum個のスキャンまで読み飛ばす
void SlamLauncher::skipData(int num) {
  Scan2D scan;
  bool eof = sreader.loadScan(0, scan);
  for (int i=0; !eof && i<num; i++) {       // num個空読みする
    eof = sreader.loadScan(0, scan);
  }
}

///////// オドメトリのよる地図構築 //////////

void SlamLauncher::mapByOdometry(Scan2D *scan) {
//  Pose2D &pose = scan->pose;               // スキャン取得時のオドメトリ位置
  Pose2D pose;
  Pose2D::calRelativePose(scan->pose, ipose, pose);
  vector<LPoint2D> &lps = scan->lps;       // スキャン点群
  vector<LPoint2D> glps;                   // 地図座標系での点群
  for (size_t j=0; j<lps.size(); j++) {
    LPoint2D &lp = lps[j];
    LPoint2D glp;
    pose.globalPoint(lp, glp);             // センサ座標系から地図座標系に変換
    glps.emplace_back(glp);
  }

  // 点群地図pcmapにデータを格納
  pcmap->addPose(pose);
  pcmap->addPoints(glps);
  pcmap->makeGlobalMap();

  printf("Odom pose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);
}

////////// スキャン描画 ////////

void SlamLauncher::showScans() {
  mdrawer.initGnuplot();
  mdrawer.setRange(6);                     // 描画範囲。スキャンが6m四方の場合
  mdrawer.setAspectRatio(-0.9);            // x軸とy軸の比（負にすると中身が一定）

  ScanPointResampler spres;

  size_t cnt = 0;                          // 処理の論理時刻
  if (startN > 0)
    skipData(startN);                      // startNまでデータを読み飛ばす

  Scan2D scan;
  bool eof = sreader.loadScan(cnt, scan);
  while(!eof) {
//    spres.resamplePoints(&scan);         // コメントアウトはずせば、スキャン点間隔を均一にする。
 
    // 描画間隔をあける
#ifdef _WIN32
    Sleep(100);                            // WindowsではSleep
#elif __linux__
    usleep(100000);                        // Linuxではusleep
#endif

    mdrawer.drawScanGp(scan);              // スキャン描画

    printf("---- scan num=%lu ----\n", cnt);
    eof = sreader.loadScan(cnt, scan);
    ++cnt;
  }
  sreader.closeScanFile();
  printf("SlamLauncher finished.\n");
}

//////// スキャン読み込み /////////

bool SlamLauncher::setFilename(char *filename) {
  bool flag = sreader.openScanFile(filename);        // ファイルをオープン

  return(flag);
}

////////////

void SlamLauncher::customizeFramework() {
  fcustom.setSlamFrontEnd(&sfront);
  fcustom.makeFramework();
//  fcustom.customizeG();                         // 退化の対処をしない
//  fcustom.customizeH();                         // 退化の対処をする
  fcustom.customizeI();                           // ループ閉じ込みをする

  pcmap = fcustom.getPointCloudMap();           // customizeの後にやること
}
