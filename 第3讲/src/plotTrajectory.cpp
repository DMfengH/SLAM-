#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>   // 这里面都是些什么东西？？？

using namespace std;
using namespace Eigen;

string trajectory_file = "../trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);    // Eigen自带的对齐方式有什么好处？？？

int main(int argc, char **argv)
{
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;

  ifstream fin(trajectory_file);
  if (!fin){
    cout << "在这个地方没有找到轨迹文件： " << trajectory_file << endl;
    return 1;
  }

  while (!fin.eof()){
    double time, tx,ty,tz,qx,qy,qz,qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    // Isometry3d Twr(Quaterniond(qw, qx, qy, qz));     // 可以用旋转初始化，但不可以用平移初始化
    Isometry3d Twr = Isometry3d::Identity();  // 一个4*4单位阵
    Twr.rotate(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }

  cout << "总计读入数据点： "<<  poses.size() <<"\n";

  DrawTrajectory(poses);
  return 0; 
}

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses)
{
  // 显示窗口
  pangolin::CreateWindowAndBind("轨迹视图", 1024, 768); 
  glEnable(GL_DEPTH_TEST);  //开启深度检测，该功能会使得pangolin只会绘制朝向镜头的那一面像素点，避免容易混淆的透视关系出现，
  glEnable(GL_BLEND);       // 开启混合模式
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //混合函数：源混合因子和目标混合因子

  // 摄像机
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),  // 参数依次为观察相机的图像高度、宽度、4个内参以及最近和最远视距【用于移动相机时，对观察对象进行透视变换】
    pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)       // 参数依次为相机所在的位置，相机所看的视点位置，相机哪一个轴朝上。
  );

  // 交互式视图
  // setbound:视图在窗口中的范围0-1，前四个参数是上下左右，最后一个应该是长宽比？？？
  pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));

  // 绘制轨迹
  while(pangolin::ShouldQuit() == false){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);   // 分别清空色彩缓存和深度缓存并激活之前设定好的视窗对象,否则视窗内会保留上一帧的图形
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    for (size_t i = 0; i <poses.size(); ++i){
      Vector3d Ow = poses[i].translation();
      // 通过变换矩阵poses[i]， 把三个坐标轴变换到对应位置
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1,0,0));
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0,1,0));
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0,0,1));

      glBegin(GL_LINES);    // 绘制方式，GL_LINES,以线的方式绘制，即每两个点作为一条线，第一个点和第二个点是一条线，第三个点和第四个点是一条线。。。。
      // 变换后的x轴
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Xw[0], Xw[1], Xw[2]);
      // 变换后的y轴
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Yw[0], Yw[1], Yw[2]);
      // 变换后的z轴
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(Zw[0], Zw[1], Zw[2]);
      glEnd();  // 应该是和glBegin对应，标志这个绘制方式的结束。
    }

    for (size_t i = 0; i <poses.size(); ++i){
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i+1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }

    // 绘制完成，刷新显示
    pangolin::FinishFrame();
    usleep(5000);
  }
}

