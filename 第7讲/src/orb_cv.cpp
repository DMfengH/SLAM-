#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include<unistd.h>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  close(2);   // 关闭文件描述符2【0,是标准输入，1是标准输出，2是标准错误】【不关闭2，opcv会打印一堆信息】
  
  if (argc != 3){
    cout << "usage: feature_extraction img1 img2" << endl;
    return 1;
  }

  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  assert(img_1.data != nullptr && img_2.data != nullptr);

  // KeyPoint 的 6个属性：坐标，半径大小，角度，response，金字塔层级，class_id
  vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;          // 每一行是一个关键点的描述子，所以有多少关键点就有多少行
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

  Mat outimg1;
  drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("ORB features", outimg1);
  waitKey(0);

  vector<DMatch> matches;
  t1 = chrono::steady_clock::now();
  matcher->match(descriptors_1, descriptors_2, matches);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
  cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;

  // 一个std里的函数，取容器中的最大和最小值
  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  std::vector<DMatch> good_matches;
  for (int i = 0; i < descriptors_1.rows; i++){     // 每一行，是一个关键点对应的描述子
    if (matches[i].distance <= max(2*min_dist, 30.0)){
      good_matches.push_back(matches[i]);
    }
  }

  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);

  imshow("all matches", img_match);
  imshow("good matches", img_goodmatch);
  waitKey(0);


}