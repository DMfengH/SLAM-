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

// 把三个参数作为一个参数块的。
struct NUM_CURVE_FITTING_COST : public ceres::SizedCostFunction<1,3>{

  NUM_CURVE_FITTING_COST(double x, double y) : _x(x), _y(y){}
  virtual ~NUM_CURVE_FITTING_COST(){};

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
    problem.AddResidualBlock(                                   // 残差块
      new NUM2_CURVE_FITTING_COST(x_data[i],y_data[i]),
      nullptr,      // 核函数     
      a,b,c           // 待估计参数
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