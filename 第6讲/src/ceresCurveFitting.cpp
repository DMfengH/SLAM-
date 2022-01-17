#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {} 
  
  template<typename T>
  bool operator()(const T *const abc, T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
    return true;
  }

  const double _x, _y;
};

// 第一个模板参数是rediduals的大小，
// 后面的参数代表优化参数块，有几个模板参数，就有几个优化参数块，parameter的第一维就有多大，
// 模板参数的数值代表优化参数块的大小，即parameter第二维的大小
// jacobians[i], is an
// array that contains num_residuals_* parameter_block_sizes_[i]
// elements. Each jacobian block is stored in row-major order, i.e.,
//
//   jacobians[i][r*parameter_block_size_[i] + c] =
//                              d residual[r] / d parameters[i][c]
//
// g2o中是要设置优化变量的更新方式的，而ceres没有设置，是怎么更新的优化变量呢？？？
// ceres的优化变量更新方式设置，是在problem的AddParameterBlock的LocalParameterization中！！！

struct NUM_CURVE_FITTING_COST : public ceres::SizedCostFunction<1,3>{

  NUM_CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}
  virtual ~NUM_CURVE_FITTING_COST(){};
  
  // 这三个参数的类型是固定的？ Evaluate是抽象基类的纯虚函数
  virtual bool Evaluate(double const* const* parameters, double *residuals, double** jacobians) const
  {

    const double a = parameters[0][0];
    const double b = parameters[0][1];
    const double c = parameters[0][2];

    residuals[0] = _y - ceres::exp(a*_x*_x + b*_x +c);

    if (jacobians){
      if (jacobians[0]){
        jacobians[0][0] = -_x*_x* ceres::exp(a*_x*_x + b*_x + c); 
        jacobians[0][1] = -_x* ceres::exp(a*_x*_x + b*_x + c); 
        jacobians[0][2] = -ceres::exp(a*_x*_x + b*_x + c); 
      }
    }

    return true;
  }

  const double _x, _y;
};

// 把三个参数分别作为三个参数块的。
struct NUM2_CURVE_FITTING_COST : public ceres::SizedCostFunction<1,1,1,1>{

  NUM2_CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}
  virtual ~NUM2_CURVE_FITTING_COST(){};

  virtual bool Evaluate(double const* const* parameters, double *residuals, double** jacobians) const
  {

    const double a = parameters[0][0];
    const double b = parameters[1][0];
    const double c = parameters[2][0];

    residuals[0] = _y - ceres::exp(a*_x*_x + b*_x +c);

    if (jacobians){
      if (jacobians[0]){
        jacobians[0][0] = -_x*_x* ceres::exp(a*_x*_x + b*_x + c); 
        jacobians[1][0] = -_x* ceres::exp(a*_x*_x + b*_x + c); 
        jacobians[2][0] = -ceres::exp(a*_x*_x + b*_x + c); 
      }
    }

    return true;
  }

  const double _x, _y;
};


int main(int argc, char **argv)
{
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

  // 可以把三个参数当做一个参数块，也可以把三个参数分别作为三个参数块【但是每一个参数块都得是数组】
  // double abc[3] = {ae, be, ce};
  double a[1] = {ae};
  double b[1] = {be};
  double c[1] = {ce};

  ceres::Problem problem;
  for (int i=0; i <N; i++){                                     // 循环添加所有残差
    // 自动求导
    // problem.AddResidualBlock(
    //   new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
    //     new CURVE_FITTING_COST(x_data[i], y_data[i])
    //   ),
    //   nullptr,
    //   abc
    // );

    // 手动给出雅克比求导。
    // 用户在调用AddResidualBlock( )时其实已经隐式地向Problem传递了参数模块，
    // 但在一些情况下，需要用户显示地向Problem传入参数模块（通常出现在需要对优化参数进行重新参数化的情况）。
    // Ceres提供了Problem::AddParameterBlock( )函数用于用户显式传递参数模块:
    // 例： void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)
    // 函数里面设置：更新方法、优化参数的实际维度、内部优化时参数的维度、这两种参数的雅克比矩阵


    problem.AddResidualBlock(                                   // 残差块
      new NUM2_CURVE_FITTING_COST(x_data[i],y_data[i]),         // CostFunction *cost_function, 
      nullptr,                                                  // LossFunction *loss_function,   
      a,b,c                                                     // const vector<double *> parameter_blocks 、或者 double *x0, double *x1, ...
    );
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;    // 增量方程求解方式
  options.minimizer_progress_to_stdout = true ; 

  ceres::Solver::Summary summary;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);    // 开始优化
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);

  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  cout << summary.BriefReport() << endl;
  cout << "estimated a,b,c = " ;
  // for (auto a:abc) cout<< a << " ";
  cout << *a <<" " << *b << " " << *c << endl;
  cout << endl;
}