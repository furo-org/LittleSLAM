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
 * @file main.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SlamLauncher.h"

int main(int argc, char *argv[]) {
  bool scanCheck=false;              // スキャン表示のみか
  bool odometryOnly=false;           // オドメトリによる地図構築か
  char *filename;                    // データファイル名
  int startN=0;                      // 開始スキャン番号

  if (argc < 2) {
    printf("Error: too few arguments.\n");
    return(1);
  }

  // コマンド引数の処理
  int idx=1;
  // コマンドオプションの解釈（'-'のついた引数）
  if (argv[1][0] == '-') {
    for (int i=1; ; i++) {
      char option = argv[1][i];
      if (option == NULL)
        break;
      else if (option == 's')        // スキャン表示のみ
        scanCheck = true;
      else if (option == 'o')        // オドメトリによる地図構築
        odometryOnly = true;
    }
    if (argc == 2) {
      printf("Error: no file name.\n");
      return(1);
    }
    ++idx;
  }
  if (argc >= idx+1)                 // '-'ある場合idx=2、ない場合idx=1
    filename = argv[idx];
  if (argc == idx+2)                 // argcがidxより2大きければstartNがある
    startN = atoi(argv[idx+1]);
  else if (argc >= idx+2) {
    printf("Error: invalid arguments.\n");
    return(1);
  }
  
  printf("SlamLauncher: startN=%d, scanCheck=%d, odometryOnly=%d\n", startN, scanCheck, odometryOnly);
  printf("filename=%s\n", filename);

  // ファイルを開く
  SlamLauncher sl;
  bool flag = sl.setFilename(filename);
  if (!flag)
    return(1);

  sl.setStartN(startN);              // 開始スキャン番号の設定

  // 処理本体
  if (scanCheck)
    sl.showScans();
  else {                             // スキャン表示以外はSlamLauncher内で場合分け
    sl.setOdometryOnly(odometryOnly);
    sl.customizeFramework();
    sl.run();
  }

  return(0);
}
