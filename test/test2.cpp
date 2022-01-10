#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;



void add(int *a)
{
  for (int i = 0; i< 3 ;  i++){
    a[i] = a[i]+1;
  }
}

int main(int argc, char **argv)
{
  int a[3] = {1,2,3};
  add(a);
  for (int i = 0; i< 3 ;  i++){
    cout << a[i] << endl;
  }
  
}