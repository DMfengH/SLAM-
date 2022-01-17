#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_unary_edge.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
#include <chrono>
#include <sophus/se3.hpp>
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

void pose_estimation_3d3d(
  const vector<Point3f> &pts1,
  const vector<Point3f> &pts2,
  Mat &R, Mat &t
);

void bundleAdjustmentG2O(
  const vector<Point3f> &points_3d,
  const vector<Point3f> &points_2d,
  Mat &R, Mat &t
);


void bundleAdjustmentCeres(
  const vector<Point3f> &points_3d,
  const vector<Point3f> &points_2d,
  Mat &R, Mat &t
);



// class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

//   virtual void setToOriginImpl() override {
//     _estimate = Sophus::SE3d();
//   }

//   /// left multiplication on SE3
//   virtual void oplusImpl(const double *update) override {
//     Eigen::Matrix<double, 6, 1> update_eigen;
//     update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
//     _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
//   }

//   virtual bool read(istream &in) override {}

//   virtual bool write(ostream &out) const override {}
// };

// /// g2o edge
// class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

//   EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

//   virtual void computeError() override {
//     const VertexPose *pose = static_cast<const VertexPose *> ( _vertices[0] );
//     _error = _measurement - pose->estimate() * _point;
//   }

//   virtual void linearizeOplus() override {
//     VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
//     Sophus::SE3d T = pose->estimate();
//     Eigen::Vector3d xyz_trans = T * _point;
//     _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
//     _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
//   }

//   bool read(istream &in) {}

//   bool write(ostream &out) const {}

// protected:
//   Eigen::Vector3d _point;
// };



struct BAceres: public ceres::LocalParameterization{
public:
  virtual ~BAceres() {}
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


struct BA_CERES_COST : public ceres::SizedCostFunction<3,6>{
  BA_CERES_COST(const Eigen::Vector3d point1, const Eigen::Vector3d point2): 
                _point1(point1), _point2(point2) {}

  virtual ~BA_CERES_COST(){};

  virtual bool Evaluate(double const* const* parameters, double *residuals, double** jacobians) const
  {
    Sophus::SE3d T21 = Sophus::SE3d::exp(Vector6d(parameters[0]));

    Eigen::Vector3d res = _point2 - T21 * _point1;
    residuals[0] = res[0]; 
    residuals[1] = res[1];
    residuals[2] = res[2];

    Eigen::Vector3d transPoint1 = T21 * _point1; 
    Eigen::MatrixXd jac(3,6);
    jac.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    jac.block<3,3>(0,3) = Sophus::SO3d::hat(transPoint1);
    jac.transposeInPlace();
    jac.resize(1,18);
    
    if(jacobians){
      if(jacobians[0]){
        for(int i=0; i< 18;++i){
          jacobians[0][i] = jac(i);
        }
        return true;        
      }
    }
  }

private:
  const Eigen::Vector3d _point1;
  const Eigen::Vector3d _point2;
};