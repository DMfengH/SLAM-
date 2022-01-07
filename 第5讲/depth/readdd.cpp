#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main(int argc, char **argv) {
    vector<cv::Mat> depthImgs;    // 彩色图和深度图

    depthImgs.push_back(cv::imread("../1.pgm", -1)); // 使用-1读取原始图像
    cout << depthImgs[0] << endl;
    return 0;
}

