#include <iostream>
#include <Eigen/Core>

using namespace std;


int main()
{
  // 可变大小的MatrixXd 可以resize
  Eigen::MatrixXd jac(3,6);
  jac.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
  jac.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
  cout << "jac: \n" << jac << endl;
  jac.transposeInPlace();
  jac.resize(1,18);
  cout << "resize(1,18): \n" << jac << endl;
}