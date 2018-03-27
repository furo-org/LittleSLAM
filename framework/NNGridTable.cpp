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
 * @file NNGridTable.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "NNGridTable.h"

using namespace std;

////////////

// 格子テーブルにスキャン点lpを登録する
void NNGridTable::addPoint(const LPoint2D *lp) {
  // テーブル検索のインデックス計算。まず、対象領域内にあるかチェックする。
  int xi = static_cast<int>(lp->x/csize) + tsize;
  if (xi < 0 || xi > 2*tsize)                      // 対象領域の外
    return;
  int yi = static_cast<int>(lp->y/csize) + tsize;
  if (yi < 0 || yi > 2*tsize)                      // 対象領域の外
    return;

  size_t idx = static_cast<size_t>(yi*(2*tsize +1) + xi);   // テーブルのインデックス
  table[idx].lps.push_back(lp);                             // 目的のセルに入れる
}

///////////

// スキャン点clpをpredPoseで座標変換した位置に最も近い点を格子テーブルから見つける
const LPoint2D *NNGridTable::findClosestPoint(const LPoint2D *clp, const Pose2D &predPose) {
  LPoint2D glp;                           // clpの予測位置
  predPose.globalPoint(*clp, glp);         // relPoseで座標変換

  // clpのテーブルインデックス。対象領域内にあるかチェックする。
  int cxi = static_cast<int>(glp.x/csize) + tsize;
  if (cxi < 0 || cxi > 2*tsize)
    return(nullptr);
  int cyi = static_cast<int>(glp.y/csize) + tsize;
  if (cyi < 0 || cyi > 2*tsize)
    return(nullptr);

  size_t pn=0;                            // 探したセル内の点の総数。確認用
  double dmin=1000000;
  const LPoint2D *lpmin = nullptr;        // 最も近い点（目的の点）
  double dthre=0.2;                       // これより遠い点は除外する[m]
  int R=static_cast<int>(dthre/csize);

  // ±R四方を探す
  for (int i=-R; i<=R; i++) {
    int yi = cyi+i;                       // cyiから広げる
    if (yi < 0 || yi > 2*tsize)
      continue;
    for (int j=-R; j<=R; j++) {
      int xi = cxi+j;                     // cxiから広げる
      if (xi < 0 || xi > 2*tsize)
        continue;

      size_t idx = yi*(2*tsize+1) + xi;             // テーブルインデックス
      NNGridCell &cell = table[idx];                // そのセル
      vector<const LPoint2D*> &lps = cell.lps;      // セルがもつスキャン点群
      for (size_t k=0; k<lps.size(); k++) {
        const LPoint2D *lp = lps[k];
        double d = (lp->x - glp.x)*(lp->x - glp.x) + (lp->y - glp.y)*(lp->y - glp.y);

        if (d <= dthre*dthre && d < dmin) {         // dthre内で距離が最小となる点を保存
          dmin = d;
          lpmin = lp;
        }
      }
      pn += lps.size();
    }
  }
//  printf("pn=%d\n", pn);                 // 探したセル内の点の総数。確認用

  return(lpmin);
}

////////////

// 格子テーブルの各セルの代表点を作ってpsに格納する。
void NNGridTable::makeCellPoints(int nthre, vector<LPoint2D> &ps) {
  // 現状はセル内の各点のスキャン番号の平均をとる。
  // スキャン番号の最新値をとる場合は、その部分のコメントをはずし、
  // 平均とる場合（2行）をコメントアウトする。

  size_t nn=0;                           // テーブル内の全セル数。確認用
  for (size_t i=0; i<table.size(); i++) {
    vector<const LPoint2D*> &lps = table[i].lps;      // セルのスキャン点群
    nn += lps.size();
    if (lps.size() >= nthre) {           // 点数がnthreより多いセルだけ処理する
      double gx=0, gy=0;                 // 点群の重心位置
      double nx=0, ny=0;                 // 点群の法線ベクトルの平均
      int sid=0;
      for (size_t j=0; j<lps.size(); j++) {
        const LPoint2D *lp = lps[j];
        gx += lp->x;                     // 位置を累積
        gy += lp->y;
        nx += lp->nx;                    // 法線ベクトル成分を累積
        ny += lp->ny;
        sid += lp->sid;                  // スキャン番号の平均とる場合
//        if (lp->sid > sid)             // スキャン番号の最新値とる場合
//          sid = lp->sid;
//        printf("sid=%d\n", lp->sid);
      }
      gx /= lps.size();                  // 平均
      gy /= lps.size();
      double L = sqrt(nx*nx + ny*ny);
      nx /=  L;                          // 平均（正規化）
      ny /=  L;
      sid /= lps.size();                 // スキャン番号の平均とる場合

      LPoint2D newLp(sid, gx, gy);       // セルの代表点を生成
      newLp.setNormal(nx, ny);           // 法線ベクトル設定
      newLp.setType(LINE);               // タイプは直線にする
      ps.emplace_back(newLp);            // psに追加
    }
  }

//  printf("nn=%d\n", nn);               // テーブル内の全セル数。確認用
}
