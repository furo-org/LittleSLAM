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
 * @file FrameworkCustomizer.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef FRAMEWORK_CUSTOMIZER_H_
#define FRAMEWORK_CUSTOMIZER_H_

#include <vector>
#include "MyUtil.h"
#include "RefScanMaker.h" 
#include "RefScanMakerBS.h" 
#include "RefScanMakerLM.h" 
#include "DataAssociator.h" 
#include "DataAssociatorLS.h" 
#include "DataAssociatorGT.h" 
#include "CostFunction.h" 
#include "CostFunctionED.h" 
#include "CostFunctionPD.h" 
#include "PoseOptimizer.h" 
#include "PoseOptimizerSD.h" 
#include "PoseOptimizerSL.h" 
#include "PointCloudMap.h" 
#include "PointCloudMapBS.h" 
#include "PointCloudMapGT.h" 
#include "PointCloudMapLP.h" 
#include "LoopDetector.h" 
#include "LoopDetectorSS.h" 
#include "ScanPointResampler.h"
#include "ScanPointAnalyser.h"
#include "PoseEstimatorICP.h" 
#include "PoseFuser.h" 
#include "ScanMatcher2D.h" 
#include "SlamFrontEnd.h" 

class FrameworkCustomizer
{
  // フレームワーク改造用の部品
  RefScanMakerBS rsmBS;
  RefScanMakerLM rsmLM;
  DataAssociatorLS dassLS;
  DataAssociatorGT dassGT;
  CostFunctionED cfuncED;
  CostFunctionPD cfuncPD;
  PoseOptimizerSD poptSD;
  PoseOptimizerSL poptSL;
  PointCloudMapBS pcmapBS;
  PointCloudMapGT pcmapGT;
  PointCloudMapLP pcmapLP;
  PointCloudMap *pcmap;            // SlamLauncherで参照するためメンバ変数にする
  LoopDetector lpdDM;              // ダミー。何もしない
  LoopDetectorSS lpdSS;
  ScanPointResampler spres;
  ScanPointAnalyser spana;

  PoseEstimatorICP poest;
  PoseFuser pfu;
  ScanMatcher2D smat;
  SlamFrontEnd *sfront;

public:
  FrameworkCustomizer() : pcmap(nullptr) {
  }

  ~FrameworkCustomizer() {
  }

  void setSlamFrontEnd(SlamFrontEnd *f) {
    sfront = f;
  }

  PointCloudMap *getPointCloudMap() {
    return(pcmap);
  }

//////

  void makeFramework();
  void customizeA();
  void customizeB();
  void customizeC();
  void customizeD();
  void customizeE();
  void customizeF();
  void customizeG();
  void customizeH();
  void customizeI();
};

#endif
