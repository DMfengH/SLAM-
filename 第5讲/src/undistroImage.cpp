#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;

string image_file = "../distorted.png";

void makeImageUndistort(const cv::Mat &distort, cv::Mat &undistort, const vector<double> &kp , const vector<double> &fc);


int main(int argc, char **argv)
{
  double k1 = -0.2834081, k2=0.07395907, p1 =0.00019359, p2 = 1.76187114e-05;
  double fx = 458.654 , fy = 457.296, cx = 367.215, cy =248.375;

  vector<double> kp = {k1, k2, p1, p2} , fc = {fx, fy, cx, cy};

  cv::Mat image = cv::imread(image_file, 0);
  int rows= image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);

  makeImageUndistort(image, image_undistort,kp, fc);

  cv::imshow("distroted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey(0);

}


void makeImageUndistort(const cv::Mat &distort, cv::Mat &undistort, const vector<double> &kp , const vector<double> &fc)
{
  if (kp.size() != 4){
    cout << "畸变参数数量应该是4个" << endl;
    return ;
  }

  if (fc.size() != 4){
    cout << "相机内参数量应该是4个" << endl;
    return ;
  }

  double k1 = kp[0], k2 = kp[1], p1 = kp[2], p2 = kp[3];
  double fx = fc[0], fy = fc[1], cx = fc[2], cy = fc[3];

  int rows = distort.rows, cols = distort.cols;

  for (int v= 0; v < rows; v++){
    for (int u = 0;u <cols; ++u){
      double x = (u-cx) / (fx) , y = (v - cy) / fy;   // 归一化平面的坐标
      double r = sqrt(x*x + y*y);
      double x_distorted = x*(1 + k1*r*r + k2*r*r*r*r) + 2*p1*x*y + p2*(r*r+2*x*x);    // 归一化平面去畸变后的对应坐标
      double y_distorted = y*(1 + k1*r*r + k2*r*r*r*r) + p1*(r*r + 2*y*y) + 2*p2*x*y;
      double u_distorted = fx*x_distorted + cx;     // 图像中去畸变后的对应坐标
      double v_distorted = fy*y_distorted + cy;

      if (u_distorted >= 0 && v_distorted >=0 && u_distorted < cols && v_distorted < rows){
        undistort.at<uchar>(v,u) = distort.at<uchar>((int) v_distorted, (int) u_distorted);
      } else {
        undistort.at<uchar>(v,u) = 0;
      }
    }
  }
  
  cout << "去畸变完成。" << endl;
}