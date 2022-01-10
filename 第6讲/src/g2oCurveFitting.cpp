#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

// 顶点：需要优化的参数
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 重置函数：【父类的父类中的纯虚函数，并且在父类中没有重写】
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0;
  }

  // 更新函数： 当变量是旋转矩阵时，要修改成对应的更新方式【即，左乘更新】  【这个也是父类的父类中的纯虚函数】
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);       
  }

  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}
};


// 边【y是测量值】：误差计算、雅克比计算
// 第一个模板参数 1 是表示误差向量和信息矩阵的维度的，第二个模板参数 double 是表示测量值类型的【类型也就包括了维度信息】
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {   
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 这里只给定x值，y是通过edge->setMeasurement()添加
  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  virtual void computeError() override {
    // _vertices 是 std::vector<Vertex*>  ，即边所连接的顶点
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]); 
    const Eigen::Vector3d abc = v->estimate();

    // 下面这两个是一样的，_error是一个只有一列的Eigen::Matrix
    // _error(0,0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1,0) * _x + abc(2,0));    
    _error[0] = _measurement - std::exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);     // 
  
  }

  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}

public:
  double _x;

};


// 边【没有测量值】：x,y值都作为边的属性，不提供测量值。【设置成测量值和作为边的属性，有什么区别呢？？】
class xyCurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 这里给定x值和y值。
  xyCurveFittingEdge(double x, double y) : BaseUnaryEdge(), _x(x), _y(y) {}

  virtual void computeError() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
 
    _error[0] = _y - std::exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);     // 
  
  }

  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double z = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * z;
    _jacobianOplusXi[1] = -_x * z;
    _jacobianOplusXi[2] = -z;
  }

  virtual bool read(istream &in) {}
  virtual bool write(ostream &out) const {}

public:
  double _x, _y;

};


int main(int argc, char **argv){

  double ar = 1.0, br=2.0, cr = 1.0;
  double ae = 2.0, be=-1.0, ce = 5.0;
  int N = 100;
  double w_sigma = 1.0;
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;

  vector<double> x_data, y_data;
  for (int i =0; i <N; i++){
    double x= i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma));
  }

  // g2o::BlockSolverTraits<p, l> 这个里面就是typedef了几个类型的Eigen::matrix【其中包括下面的BlockSolverType::PoseMatrixType】： p代表位姿维度，l代表路标维度
  // 猜测：对于一元边来说，只使用了p，没有用l，所以l定义成多少，都无所谓。
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;     
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
  );
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);
  
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));
  v->setId(0);
  optimizer.addVertex(v);

  for(int i = 0; i <N;i++){
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    // xyCurveFittingEdge *xyedge = new xyCurveFittingEdge(x_data[i], y_data[i]);    
    edge->setId(i);
    edge->setVertex(0, v);     // 边连接的第一个顶点是v，一元边就一个顶点
    edge->setMeasurement(y_data[i]);
    // xyedge->setMeasurement(0);          // 这个设置成多少都无所谓，因为xyedge没有使用测量值。【对应xyCurveFittingEdge】
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
    optimizer.addEdge(edge);

  }

  cout << "start optimization " << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds ." << endl;

  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;

}