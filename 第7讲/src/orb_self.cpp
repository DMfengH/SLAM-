#include "../include/orb_pattern.h"


int main(int argc, char **argv) {

  // close(2);    // 这个程序如果关掉文件描述符2，cv::imshow 就会卡住，很奇怪，orb_cv.cpp 中就不会。
  // load image
  cv::Mat first_image = cv::imread(argv[1],  0);
  cv::Mat second_image = cv::imread(argv[2],  0);
  assert(first_image.data != nullptr && second_image.data != nullptr);
  // detect FAST keypoints1 using threshold=40
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  vector<cv::KeyPoint> keypoints1;
  cv::FAST(first_image, keypoints1, 40);
  vector<DescType> descriptor1;
  ComputeORB(first_image, keypoints1, descriptor1);

  // same for the second
  vector<cv::KeyPoint> keypoints2;
  vector<DescType> descriptor2;
  cv::FAST(second_image, keypoints2, 40);
  ComputeORB(second_image, keypoints2, descriptor2);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "extract ORB cost = " << time_used.count() << " seconds. " << endl;

  // find matches
  vector<cv::DMatch> matches;
  t1 = chrono::steady_clock::now();
  BfMatch(descriptor1, descriptor2, matches);
  t2 = chrono::steady_clock::now();
  time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "match ORB cost = " << time_used.count() << " seconds. " << endl;
  cout << "matches: " << matches.size() << endl;

  // plot the matches
  cv::Mat image_show;
  cv::drawMatches(first_image, keypoints1, second_image, keypoints2, matches, image_show);
  cv::imshow("matches", image_show);
  // cv::imwrite("matches.png", image_show);
  cv::waitKey(0);

  cout << "done." << endl;
  return 0;
}


void ComputeORB(const cv::Mat &img, vector<cv::KeyPoint> &keypoints, vector<DescType> & descriptors )
{
  const int half_patch_size = 8;
  const int half_boundary = 16;
  int bad_points = 0;
  for (auto &kp:keypoints){
    if(kp.pt.x < half_boundary || kp.pt.y < half_boundary || kp.pt.x >= img.cols - half_boundary || kp.pt.y >=img.rows - half_boundary){
      bad_points++;
      descriptors.push_back({});    // 会放入一个默认初始化的。
      continue;
    }
    float m01 = 0;
    float m10 = 0;

    for (int dx = -half_patch_size; dx < half_patch_size; ++dx){
      for (int dy = -half_patch_size; dy <half_patch_size; ++dy){
        uchar pixel = img.at<uchar>(kp.pt.y + dy, kp.pt.x + dx);
        m01 += dx * pixel;
        m10 += dy * pixel;
      }
    }

    float m_sqrt = sqrt(m01 * m01 + m10 * m10);
    float sin_theta = m01 / m_sqrt;
    float cos_theta = m10 / m_sqrt;

    DescType desc(8, 0);
    for( int i = 0; i < 8 ; i++){
      uint32_t d = 0;
      for (int k = 0; k < 32; k++){
        int idx_pq = i* 32 + k;
        cv::Point2f p(ORB_pattern[idx_pq * 4], ORB_pattern[idx_pq * 4 + 1]);
        cv::Point2f q(ORB_pattern[idx_pq * 4 + 2], ORB_pattern[idx_pq * 4 + 3]);

        cv::Point2f pp = cv::Point2f(cos_theta * p.x - sin_theta * p.y, sin_theta * p.x + cos_theta * p.y) + kp.pt;
        cv::Point2f qq = cv::Point2f(cos_theta * q.x - sin_theta * q.y, sin_theta * q.x + cos_theta * q.y) + kp.pt;

        if(img.at<uchar>(pp.y, pp.x) < img.at<uchar>(qq.y, qq.x)){
          d |= 1 << k;
        }
      }
      desc[i] = d;
    }
    descriptors.push_back(desc);
  }
  cout << "bad / total: " << bad_points << "/" << keypoints.size() << endl;
}


void BfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches)
{
  const int d_max = 40;
  for(size_t j = 0; j < desc1.size(); ++j){
    if (desc1[j].empty()) continue;
    cv::DMatch m{int(j), 0, 256};    // _queryIdx, _trainIdx, _distance
    for (size_t k = 0; k < desc2.size(); ++k){
      if(desc2[k].empty()) continue;
      int distance = 0;
      for(int l = 0; l <8 ; l++){
        distance += _mm_popcnt_u32(desc1[j][l] ^desc2[k][l]);
      }
      if (distance<d_max && distance <m.distance){
        m.distance  = distance;
        m.trainIdx = k;
      }
    }
    if(m.distance < d_max){
      matches.push_back(m);
    }
  }
}