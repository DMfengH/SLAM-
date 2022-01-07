#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
  cv::Mat image;
  image = cv::imread(argv[1]);
  cv::imshow("image", image);       // 这个函数会一会报错，一会正常。。。。没搞懂。。。
  cv::waitKey(0);
}