#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
// #include "extra.h" // use this if in OpenCV2


using namespace std;
using namespace cv;
using namespace Eigen;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

void pose_estimation_2d2d(
  std::vector<KeyPoint> keypoints_1,
  std::vector<KeyPoint> keypoints_2,
  std::vector<DMatch> matches,
  Mat &R, Mat &t);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);


// 这里准备自己写一个求基础矩阵的函数，，，发现不会求解最小二乘问题。。。。。[准备学习一下最优化。]
void selfFindFundamentalMat(const vector<Eigen::Vector2d> &first, const vector<Eigen::Vector2d> &second)
{
  // 模板Matrix<>需要在编译期就知道大小，所以这里要使用动态矩阵MatrixXd
  Eigen::MatrixXd EMatrix(first.size(), 9);
  for (int i = 0; i<first.size(); ++i){
    double u1 = first[i](0);
    double v1 = first[i](1);
    double u2 = second[i](0);
    double v2 = second[i](1);
    EMatrix.row(i) << u2*u1, u2*v1, u2, v2*u1, v2*v1, v2, u1, v1, 1;
  }
  // Matrix<double, 9,1> d ;
  // d << 0,0,0,0,0,0,0,0,0;
  // Matrix<double, 9,1> e = EMatrix.ldlt().solve(d);



}