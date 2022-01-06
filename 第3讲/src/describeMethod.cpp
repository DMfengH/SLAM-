#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <cmath>
using namespace std;
using namespace Eigen;

// 反对称矩阵
Matrix3d pov(Vector3d p)
{
  Matrix3d mat;
  mat << 0, -p(2), p(1),
         p(2), 0, -p(0),
         -p(1), p(0), 0;
  return mat;
}

// 四元数的加矩阵
Matrix4d quaternionADD(Vector4d p )
{
  Matrix4d mat = Matrix4d::Ones();
  mat.block(0,0,1,1) = p.block(0,0,1,1);
  mat.block(0,1,1,3) = -p.block(1,0,3,1).transpose();
  mat.block(1,0,3,1) = p.block(1,0,3,1);
  mat.block(1,1,3,3) = p(0)* Matrix3d::Identity()  + pov(p.block(1,0,3,1));

  return mat;
}

// 四元数的圈加矩阵
Matrix4d quaternionCirADD(Vector4d p )
{
  Matrix4d mat = Matrix4d::Ones();
  mat.block(0,0,1,1) = p.block(0,0,1,1);
  mat.block(0,1,1,3) = -p.block(1,0,3,1).transpose();
  mat.block(1,0,3,1) = p.block(1,0,3,1);
  mat.block(1,1,3,3) = p(0)* Matrix3d::Identity()  - pov(p.block(1,0,3,1));

  return mat;
}

// 四元数的逆
Vector4d quaInverse(Vector4d p)
{
  Vector4d p2;
  p2 << p(0), -p(1), -p(2), -p(3);
  p2 = p2 / pow(p.norm(),2);
  return p2;
}

// 四元数的旋转矩阵：通过四元数的加矩阵和圈加矩阵
Matrix3d quaternionTOmatrix(Vector4d p)
{
  Matrix3d mat;
  Matrix4d temp;
  temp = quaternionADD(p) * quaternionCirADD(quaInverse(p));
  mat = temp.block(1,1,3,3);
  return mat;
}

// 四元数的旋转矩阵：通过书中式3.39
Matrix3d quaternionTOmatrix2(Vector4d p)
{
  Matrix3d mat;
  Matrix3d temp1 = p.block(1,0,3,1) * p.block(1,0,3,1).transpose();
  Matrix3d temp2 = p(0) * p(0) * Matrix3d::Identity();
  Matrix3d temp3 = 2 * p(0) * pov(p.block(1,0,3,1));
  Matrix3d temp4 = pov(p.block(1,0,3,1)) * pov(p.block(1,0,3,1));

  mat =  temp1 + temp2  + temp3 + temp4;
  return mat;
}

// 四元数的角轴表示：向量的长度为旋转角度，不是弧度
Vector3d  quaternionTOVector(Vector4d p)
{
  double theta = 2 * acos(p(0)) / M_PI * 180;
  Vector3d axis = (p.block(1,0,3,1) / sin(theta/180*M_PI/2)).normalized() * theta;
  return axis;
}

// 旋转矩阵转为四元数
Vector4d matrixTOquaternion(Matrix3d mat)
{
  // 这里是判断输入是否为单位正交阵
  double temp = (mat * mat.transpose() - Matrix3d::Identity()).norm();
  if ( temp > 0.001 || mat.determinant() - 1 > 0.001){
    cout << "输入不是单位正交阵。\n";
  }

  double s = sqrt((mat.trace()+1)/4);
  double x = (mat(2,1) - mat(1,2))/(4*s);
  double y = (mat(0,2) - mat(2,0))/(4*s);
  double z = (mat(1,0) - mat(0,1))/(4*s);
  Vector4d qua(s,x,y,z);
  return qua;

}

/**
 * normalize是对自身的修改，返回值为void；而normalized返回一个新的矩阵。
 * 
 */

int main(int argc, char **argv)
{
  Vector4d p(0.35, 0.2, 0.3, 0.1);
  p.normalize();
  Quaterniond q(0.35, 0.2, 0.3, 0.1);   // 四元数在Eigen中，赋值时w实部在首，内部表示和输出系数时，w实部在尾
  q.normalize();
  cout.precision(3);
  cout << "四元数转矩阵： \n" << Matrix3d(q) << endl;   // 这里如果q没有归一化，得到的结果是错误的
  cout << "四元数转角轴： " << AngleAxisd(q).angle()/M_PI*180 << " " << AngleAxisd(q).axis().transpose() << endl;

  cout <<"-------------------------------------\n";
  cout << "四元数： " << p.transpose() << endl;
  cout << "四元数加号： \n" << quaternionADD(p) << endl;
  cout << "四元数圈加号： \n" << quaternionCirADD(p) << endl;
  cout << "四元数取逆： " << quaInverse(p).transpose() << endl;
  cout << "四元数对应的旋转矩阵： \n" << quaternionTOmatrix(p) << endl;   // 这个p归不归一化都一样
  cout << "四元数对应的旋转矩阵2： \n" << quaternionTOmatrix2(p) << endl; // 这个需要归一化后的p
  cout << "四元数对应的角轴： " << quaternionTOVector(p).normalized().transpose() << endl; 
  cout << "旋转矩阵对应的四元数： " << matrixTOquaternion(quaternionTOmatrix(p)).transpose() << endl; 

}