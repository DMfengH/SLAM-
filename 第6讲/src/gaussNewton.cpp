#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;

// 函数声明中写了参数默认值，定义时就不能写了。
void makeCurve(const vector<double> &parameter, vector<double> &x_data, vector<double> &y_data, const int N=100 , const double w_sigma=1);
void iterSolveGN(const vector<double> &x_data, const vector<double> &y_data, vector<double> &parameter, double w_sigma=1, const int iterations=100);
void iterSolveLM(const vector<double> &x_data, const vector<double> &y_data, vector<double> &parameter, double w_sigma=1, const int iterations=100);


int main(int argc, char **argv)
{ 
  vector<double> realparam{1.0, 2.0, 1.0};    // double ar = 1.0, br = 2.0, cr = 1.0;
  vector<double> estiparam{2.0, -1.0, 5.0};   // double ae = 2.0, be = -1.0, ce = 5.0;
  int N = 100;
  double w_sigma = 1.0;

  vector<double> x_data, y_data;
  makeCurve(realparam, x_data, y_data, N, w_sigma);

  int iterations = 200;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  iterSolveGN(x_data, y_data, estiparam, w_sigma, iterations);
  chrono::steady_clock::time_point t2 =  chrono::steady_clock::now();

  chrono::duration<double> time_used  = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout <<  "solve time cost = " << time_used.count() << " seconds. " << endl;
  

}


void makeCurve(const vector<double> &parameter, vector<double> &x_data, vector<double> &y_data, const int N , const double w_sigma)
{
  cv::RNG rng;
  double ar = parameter[0], br = parameter[1], cr = parameter[2];
  for (int i=0; i <N; ++i){
    double x = i /100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma*w_sigma));
  }

  cout << "make Curve completed" << endl;
}


/**
 * 求解除了要有x、y数据，还要有sigma
 */
void iterSolveGN(const vector<double> &x_data, const vector<double> &y_data, vector<double> &parameter, double w_sigma, const int iterations)
{
  int N = x_data.size();
  double cost = 0, lastCost = 0;
  double ae = parameter[0], be = parameter[1], ce = parameter[2];
  double inv_sigma = 1.0/ w_sigma;

  for(int iter = 0; iter < iterations; iter++){
    Matrix3d H = Matrix3d::Zero();
    Vector3d b = Vector3d::Zero();
    cost = 0;

    for(int i = 0; i <N; i++){
      double xi = x_data[i], yi= y_data[i];
      double error = yi - exp(ae*xi*xi + be*xi + ce);
      Vector3d J;
      J[0] = -xi*xi*exp(ae*xi*xi + be*xi + ce);
      J[1] = -xi*exp(ae*xi*xi + be*xi + ce);
      J[2] = -exp(ae*xi*xi + be*xi + ce);

      H += inv_sigma*inv_sigma*J*J.transpose();
      b += -inv_sigma*inv_sigma*error*J;

      cost += error*error;
    }

    Vector3d dx = H.ldlt().solve(b);    // 求解正规方程、增量方程
    if(isnan(dx[0])){
      cout << "result is nan!" << endl;
      break;
    }

    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    lastCost = cost;
    
    cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() << 
            "\t\testimated params: " << ae << ", " << be << ", " << ce << endl;
  }

  cout << "estimated abc = " << ae << ", " << be << "," << ce << endl;
  parameter[0] = ae;
  parameter[1] = be;
  parameter[2] = ce;
}

/** 
 * 没搞出来
 */
void iterSolveLM(const vector<double> &x_data, const vector<double> &y_data, vector<double> &parameter,  double u, double w_sigma, const int iterations)
{
  
}