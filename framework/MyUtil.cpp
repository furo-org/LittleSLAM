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
 * @file MyUtil.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "MyUtil.h"
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

/////////

// 角度の加算。-180から180に正規化
int MyUtil::add(int a1, int a2) {
  int sum = a1 + a2;
  if (sum < -180)
    sum += 360;
  else if (sum >= 180)
    sum -= 360;

  return(sum);
}

// 角度の加算。-180から180に正規化
double MyUtil::add(double a1, double a2) {
  double sum = a1 + a2;
  if (sum < -180)
    sum += 360;
  else if (sum >= 180)
    sum -= 360;

  return(sum);
}

// 角度の加算（ラジアン）。-piからpiに正規化
double MyUtil::addR(double a1, double a2) {
  double sum = a1 + a2;
  if (sum < -M_PI)
    sum += 2*M_PI;
  else if (sum >= M_PI)
    sum -= 2*M_PI;

  return(sum);
}

////////////

// SVDを用いた逆行列計算
Eigen::Matrix3d MyUtil::svdInverse(const Matrix3d &A) {
  size_t m = A.rows();
  size_t n = A.cols();

  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

  MatrixXd eU = svd.matrixU();
  MatrixXd eV = svd.matrixV();
  VectorXd eS = svd.singularValues();  

  MatrixXd M1(m, n);
  for (size_t i=0; i<n; i++) {
//    if (eS[i] < 1.0E-10)                   // AがSingularかどうかのチェック。今回はしない
//      return;
    for (size_t j=0; j<n; j++) {
      M1(i,j) = eU(j,i)/eS[i];
    }
  }
    
  Matrix3d IA;
  for (size_t i=0; i<n; i++) {
    for (size_t j=0; j<n; j++) {
      IA(i,j) = 0;
      for (size_t k=0; k<n; k++) 
        IA(i,j) += eV(i,k)*M1(k,j);
    }
  }

  return(IA);
}

/////////

// 2次正方行列の固有値分解
void MyUtil::calEigen2D( double (*mat)[2], double *vals, double *vec1, double *vec2) {
  double a = mat[0][0];
  double b = mat[0][1];
  double c = mat[1][0];
  double d = mat[1][1];

  double B = sqrt((a+d)*(a+d) - 4*(a*d-b*c));
  double x1 = ((a+d) + B)/2;
  double x2 = ((a+d) - B)/2;
  vals[0] = x1;                    // 固有値
  vals[1] = x2;

  double m00 = a-x1;
  double m01 = b;
  double L = sqrt(m00*m00 + m01*m01);
  vec1[0] = m01/L;                 // 固有ベクトル
  vec1[1] = -m00/L;

  m00 = a-x2;
  m01 = b;
  L = sqrt(m00*m00 + m01*m01);
  vec2[0] = m01/L;                 // 固有ベクトル
  vec2[1] = -m00/L;

/*
  // 検算
  double ax1 = a*vec1[0] + b*vec1[1];
  double ax2 = x1*vec1[0];
  double ay1 = c*vec1[0] + d*vec1[1];
  double ay2 = x1*vec1[1];
  printf("ax1=%g, ax2=%g\n", ax1, ax2);
  printf("ay1=%g, ay2=%g\n", ay1, ay2);

  double prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
  printf("prod=%g\n", prod);
*/

}

