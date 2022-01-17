#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// // #include<stdio.h>
// #include <stdlib.h>
// #include<unistd.h>
// #include <stdarg.h>
#include "sophus/se3.hpp"
// #include "sophus/so3.hpp"
#include <Eigen/Geometry>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<double, 6, 1> Vector6d;


int main()
{
  // double jacobian[30];
  // ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
  // for (int i = 0; i <50; i++){
  //   cout << jacobian[i] << " ";
  //   if ((i+1)%6 == 0 ){
  //     cout << endl;
  //   }
  // }
  Vector6d vc;
  vc << 1,0,0,0,0,0;
  Vector3d xyz(1,2,3);

  Sophus::SE3d SE3_1 = Sophus::SE3d::exp(vc);

  Eigen::Vector3d pos_cam = SE3_1 * xyz;
  cout << pos_cam << endl;
}