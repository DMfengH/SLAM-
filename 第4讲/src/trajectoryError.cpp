#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;

string groundtruth_file = "../groundtruth.txt";
string estimated_file = "../estimated.txt";

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv)
{
  TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
  TrajectoryType estimated = ReadTrajectory(estimated_file);
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  double rmse = 0;
  for( size_t i =0; i < estimated.size(); ++i){
    Sophus::SE3d p1 = estimated[i];
    Sophus::SE3d p2 = groundtruth[i];
    double error = (p2.inverse() * p1).log().norm();
    rmse += error*error;
  }
  rmse = rmse / double(estimated.size());
  rmse = sqrt(rmse);
  cout << "RMSE = " << rmse <<endl;

  DrawTrajectory(groundtruth, estimated);

}

TrajectoryType ReadTrajectory(const string &path)
{
  ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin){
    cerr<< "trajectory " << path << " not found." << endl;
    return trajectory;
  }

  while (!fin.eof()){
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >>qx >> qy >>qz >>qw;
    Sophus:: SE3d p1(Eigen::Quaterniond(qx, qy,qz,qw), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }

  return trajectory;
}

void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti)
{
  pangolin::CreateWindowAndBind("轨迹误差", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),  
    pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0) 
  );

  pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    for(size_t i = 0; i < gt.size()-1; ++i){
      
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      auto p1 = gt[i].translation();
      auto p2 = gt[i+1].translation();
      glVertex3d(p1(0),p1(1),p1(2));
      glVertex3d(p2(0),p2(1),p2(2));

      glColor3f(0.0, 1.0, 0.0);
      auto p3 = esti[i].translation();
      auto p4 = esti[i+1].translation();
      glVertex3d(p3(0),p3(1),p3(2));
      glVertex3d(p4(0),p4(1),p4(2));
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);
  }

}