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
 * @file ScanPointResampler.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "ScanPointResampler.h"

using namespace std;

/////////

void ScanPointResampler::resamplePoints(Scan2D *scan) {
  vector<LPoint2D> &lps = scan->lps;           // スキャン点群
  if (lps.size() == 0)
    return;

  vector<LPoint2D> newLps;                     // リサンプル後の点群

  dis = 0;                                     // disは累積距離
  LPoint2D lp = lps[0];
  LPoint2D prevLp = lp;
  LPoint2D np(lp.sid, lp.x, lp.y);
  newLps.emplace_back(np);                     // 最初の点は入れる
  for (size_t i=1; i<lps.size(); i++) {
    lp = lps[i];                               // スキャン点
    bool inserted=false;

    bool exist = findInterpolatePoint(lp, prevLp, np, inserted);

    if (exist) {                               // 入れる点がある
      newLps.emplace_back(np);                 // 新しい点npを入れる
      prevLp = np;                             // npが直前点になる
      dis = 0;                                 // 累積距離をリセット
      if (inserted)                            // lpの前で補間点を入れたので、lpをもう一度やる
        i--;
    }
    else
      prevLp = lp;                             // 今のlpが直前点になる
  }

  scan->setLps(newLps);

  printf("lps.size=%lu, newLps.size=%lu\n", lps.size(), newLps.size());    // 確認用
}

bool ScanPointResampler::findInterpolatePoint(const LPoint2D &cp, const LPoint2D &pp, LPoint2D &np, bool &inserted) {
  double dx = cp.x - pp.x;
  double dy = cp.y - pp.y;
  double L = sqrt(dx*dx+dy*dy);             // 現在点cpと直前点ppの距離
  if (dis+L < dthreS) {                     // 予測累積距離(dis+L)がdthreSより小さい点は削除
    dis += L;                               // disに加算
    return(false);
  }
  else if (dis+L >= dthreL) {               // 予測累積距離がdthreLより大きい点は補間せず、そのまま残す
    np.setData(cp.sid, cp.x, cp.y);
  }
  else {                                    // 予測累積距離がdthreSを超えたら、dthreSになるように補間する
    double ratio = (dthreS-dis)/L;
    double x2 = dx*ratio + pp.x;            // 少し伸ばして距離がdthreSになる位置
    double y2 = dy*ratio + pp.y;
    np.setData(cp.sid, x2, y2);
    inserted = true;                        // cpより前にnpを入れたというフラグ
  }
 
  return(true);
}

