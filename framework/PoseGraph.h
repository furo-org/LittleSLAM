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
 * @file PoseGraph.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef POSE_GRAPH_H_
#define POSE_GRAPH_H_

#include <vector>
#include "MyUtil.h"
#include "Pose2D.h"

struct PoseArc;

/////////

// ポーズグラフの頂点
struct PoseNode
{
  int nid;                       // ノードID。PoseGraphのnodesのインデックス（通し番号）
  Pose2D pose;                   // このノードのロボット位置
  std::vector<PoseArc*> arcs;    // このノードにつながるアーク

  PoseNode(): nid(-1) {
  }

  PoseNode(const Pose2D &p) : nid(-1) {
    pose = p;
  }

  ~PoseNode() {
  }

//////

  void init() {
    nid = -1;
    arcs.clear();
  }

  void setPose(const Pose2D &p) {
    pose = p;
  }
  
  void setNid(int n) {
    nid = n;
  }

  void addArc(PoseArc *a) {
    arcs.push_back(a);
  }

};

////////

// ポーズグラフの辺
struct PoseArc
{
  PoseNode *src;                      // このアークの始点側のノード
  PoseNode *dst;                      // このアークの終点側のノード
  Pose2D relPose;                     // このアークのもつ相対位置(計測値)
  Eigen::Matrix3d inf;                // 情報行列

  PoseArc(void) : src(nullptr), dst(nullptr){
  }

  PoseArc(PoseNode *s, PoseNode *d, Pose2D &rel, const Eigen::Matrix3d _inf) {
    setup(s, d, rel, _inf);
  }
  
  ~PoseArc(void) {
  }

/////////

  void setup(PoseNode *s, PoseNode *d, const Pose2D &rel, const Eigen::Matrix3d _inf) {
    src = s;
    dst = d;
    relPose = rel;
    inf = _inf;
  }

};

////////// ポーズグラフ //////////

class PoseGraph
{
private:
  static const int POOL_SIZE=100000;
  std::vector<PoseNode> nodePool;     // ノード生成用のメモリプール
  std::vector<PoseArc> arcPool;       // アーク生成用のメモリプール

public:
  std::vector<PoseNode*> nodes;       // ノードの集合
  std::vector<PoseArc*> arcs;         // アークの集合。アークは片方向のみもつ

  PoseGraph() {
    nodePool.reserve(POOL_SIZE);      // メモリプールの領域を最初に確保。vectorはサイズが変わると中身が移動するのでこうしないと危険
    arcPool.reserve(POOL_SIZE);
  }

  ~PoseGraph() {
  }
  
//////////////
  
  void reset() {
    nodes.clear();
    arcs.clear();
    nodePool.clear();
    arcPool.clear();
  }

  // ノードの生成
  PoseNode *allocNode() {
    if (nodePool.size() >= POOL_SIZE) {
      printf("Error: exceeds nodePool capacity\n");
      return(nullptr);
    }
   
    PoseNode node;
    nodePool.emplace_back(node);      // メモリプールに追加して、それを参照する。
    return(&(nodePool.back()));
  }

  // アークの生成
  PoseArc *allocArc() {
    if (arcPool.size() >= POOL_SIZE) {
      printf("Error: exceeds arcPool capacity\n");
      return(nullptr);
    }

    PoseArc arc;
    arcPool.emplace_back(arc);       // メモリプールに追加して、それを参照する。
    return(&(arcPool.back()));
  }

//////////////

  PoseNode *addNode(const Pose2D &pose);
  void addNode(PoseNode *n1, const Pose2D &pose);
  PoseNode *findNode(int nid);

  void addArc(PoseArc *arc);
  PoseArc *makeArc(int srcNid, int dstNid, const Pose2D &relPose, const Eigen::Matrix3d &cov);
  PoseArc *findArc(int srcNid, int dstNid);

  void printNodes();
  void printArcs();

};

#endif
