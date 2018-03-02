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
 * @file ScanPointAnalyser.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "ScanPointAnalyser.h"

using namespace std;
  
const double ScanPointAnalyser::FPDMIN = 0.06;      // ScanPointResampler.dthrSとずらす
const double ScanPointAnalyser::FPDMAX = 1.0;

///////////

// スキャン点の法線ベクトルを求める。また、直線、コーナ、孤立の場合分けをする。
void ScanPointAnalyser::analysePoints(vector<LPoint2D> &lps) {
  for (int i=0; i<lps.size(); i++) {
    LPoint2D &lp = lps[i];                        // スキャン点
    ptype type;
    Vector2D nL, nR, normal;
    bool flagL = calNormal(i, lps, -1, nL);        // nLはlpと左側の点で求めた法線ベクトル
    bool flagR = calNormal(i, lps, 1, nR);         // nRはlpと右側の点で求めた法線ベクトル
    nR.x = -nR.x;                                 // 符号をnLと合せる
    nR.y = -nR.y;
    if (flagL) {
      if (flagR) {                                     // 左右両側で法線ベクトルが計算可能
        if (fabs(nL.x*nR.x + nL.y*nR.y) >= costh) {    // 両側の法線が平行に近い
          type = LINE;                                 // 直線とみなす
        }
        else {                                         // 平行から遠ければ、コーナ点とみなす
          type = CORNER;
        }
        // 左右両側の法線ベクトルの平均
        double dx = nL.x+nR.x;
        double dy = nL.y+nR.y;
        double L = sqrt(dx*dx + dy*dy);
        normal.x = dx/L;
        normal.y = dy/L;
      }
      else {                       // 左側しか法線ベクトルがとれなかった
        type = LINE;
        normal = nL;
      }
    }
    else {
      if (flagR) {                 // 右側しか法線ベクトルがとれなかった
        type = LINE;
        normal = nR;
      }
      else {                       // 両側とも法線ベクトルがとれなかった
        type = ISOLATE;            // 孤立点とみなす
        normal.x = INVALID;
        normal.y = INVALID;
      }
    }
      
    lp.setNormal(normal.x, normal.y);
    lp.setType(type);
  }
}
  
  // 注目点cpの両側の点が、cpからdmin以上dmax以下の場合に、法線を計算する。
bool ScanPointAnalyser::calNormal(int idx, const vector<LPoint2D> &lps, int dir, Vector2D &normal){
  const LPoint2D &cp = lps[idx];                          // 注目点
  for (int i=idx+dir; i>=0 && i<lps.size(); i+=dir) {
    const LPoint2D &lp = lps[i];                          // cpのdir（左か右）側の点
    double dx = lp.x - cp.x;
    double dy = lp.y - cp.y;
    double d = sqrt(dx*dx + dy*dy);
    if (d>=FPDMIN && d<=FPDMAX) {                         // cpとlpの距離dが適切なら法線計算
      normal.x = dy/d;
      normal.y = -dx/d;
      return(true);
    }
      
    if (d > FPDMAX)                                       // もはやどんどん離れるので、途中でやめる
      break;
  }

  return(false);
}
