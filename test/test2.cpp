#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include<stdio.h>
// #include <stdlib.h>
#include<unistd.h>
// #include <stdarg.h>


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
  int fd;
  dup2(1,fd);
  cout <<"nihao"<< endl;
  close(1);

  // cout <<"bbbb222" << endl;      // 为什么运行这句之后，aaaa就输出不出来了？？？
  dup2(fd,1);
  cout << "aaaa";
  // close(1);


}