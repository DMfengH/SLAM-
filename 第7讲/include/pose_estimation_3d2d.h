#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <iomanip>
// #include <eigen3/Eigen/Core>/
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>        // 这个头文件和ceres头文件一起出现就会报错！！！！
// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/solver.h>
// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <chrono>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose
);

// BA by gauss-newton
void bundleAdjustmentGaussNewton(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose
);

void bundleAdjustmentCERES(
  const VecVector3d &points_3d,
  const VecVector2d &points_2d,
  const Mat &K,
  Sophus::SE3d &pose
);


class SE3dceres : public ceres::LocalParameterization {
public:
  virtual ~SE3dceres() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const{

    Sophus::SE3d SE3_1(Sophus::SE3d::exp(Vector6d(x)));
    Sophus::SE3d SE3_2(Sophus::SE3d::exp(Vector6d(delta)));

    Sophus::SE3d SE3_new = SE3_2 * SE3_1; 

    Eigen::Matrix<double, 6, 1> se3 = SE3_new.log();

    x_plus_delta[0] = se3[0];   // se3 前面对应的是平移、后面对应的是旋转
    x_plus_delta[1] = se3[1];
    x_plus_delta[2] = se3[2];
    x_plus_delta[3] = se3[3];
    x_plus_delta[4] = se3[4];
    x_plus_delta[5] = se3[5];
  }

  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const{ 
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6,6);
    return true;        
  }
  virtual int GlobalSize() const { return 6; }
  virtual int LocalSize() const { return 6; }

};


struct POSE_FITTING_COST :public ceres::SizedCostFunction<2,6>{
  POSE_FITTING_COST(const Eigen::Vector3d xyz, const Eigen::Vector2d uv, const Eigen::Matrix3d K):
                    _xyz(xyz), _uv(uv) ,_K(K){}

  virtual ~POSE_FITTING_COST(){};

  virtual bool Evaluate(double const* const* parameters, double *residuals, double** jacobians) const
  {
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    // se3中：平移在前，旋转在后
    Sophus::SE3d T21 = Sophus::SE3d::exp(Vector6d(parameters[0]));

    // 这里X、Y、Z是三维点，没有乘以_K之前的【这里写错，排查了好久】
    // Z 在乘以_K前后应该是一样的。
    Eigen::Vector3d T_xyz = T21 * _xyz;
    double X = T_xyz[0];
    double Y = T_xyz[1];
    double Z = T_xyz[2];
    double Z2 = Z * Z;

    Eigen::Vector3d KT_uv = _K * T_xyz;
    KT_uv /= KT_uv[2];
    residuals[0] = _uv[0] - KT_uv[0];
    residuals[1] = _uv[1] - KT_uv[1];

    // jacobians的第一维对应参数块，这里只有一个参数块，所以jacobian第一维只有1个
    // 第二维是 误差个数*参数个数 ，这里是2个误差，6个参数，所以第二维是12个
    if(jacobians){
      if(jacobians[0]){
        jacobians[0][0] = -fx / Z;
        jacobians[0][1] = 0;
        jacobians[0][2] = fx * X / Z2;
        jacobians[0][3] = fx * X * Y / Z2;
        jacobians[0][4] = -fx - fx * X * X / Z2;
        jacobians[0][5] = fx * Y / Z;

        jacobians[0][6] = 0;   
        jacobians[0][7] = -fy / Z;
        jacobians[0][8] = fy * Y / (Z * Z);
        jacobians[0][9] = fy + fy * Y * Y / Z2;
        jacobians[0][10] = -fy * X * Y / Z2;
        jacobians[0][11] = -fy * X / Z;
        return true;
      }  
    } 
  }

  
private:
  const Eigen::Vector3d _xyz;
  const Eigen::Vector2d _uv;
  const Eigen::Matrix3d _K;

};
