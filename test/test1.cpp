#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
  Isometry3d T(Vector3d(3,3,3));
  // T.pretranslate(Vector3d(2,2,2));
  cout << T.matrix();
  // Vector3d V(1,2,3);
  // cout << T*V;
}