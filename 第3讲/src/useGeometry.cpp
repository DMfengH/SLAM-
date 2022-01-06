#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main(int argc, char **argv)
{
  Matrix3d rotation_matrix = Matrix3d::Identity();
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0,0,1));  // Z轴旋转45°，这个角轴类型可以直接乘法。
  cout.precision(3);
  // cout << "angle axis = " << rotation_vector << endl;  // 这个东西不能输出
  cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;

  rotation_matrix = rotation_vector.toRotationMatrix();
  Vector3d v(1,0,0);
  Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle Axis) = " << v_rotated.transpose() << endl;

  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0); // 2,1,0表达的轴的顺序，2是z轴、1是y轴、0是x轴
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;


  Isometry3d T = Isometry3d::Identity();    // T 是个4*4的东西，但是可以和3维向量乘法。
  T.rotate(rotation_vector);        // 设置旋转部分
  T.pretranslate(Vector3d(1,3,4));  // 设置平移部分
  cout << "Transform matrix = \n" << T.matrix() << endl;

  Vector3d v_transformed = T * v;   //
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  Quaterniond q = Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;
  q = Quaterniond(rotation_matrix);
  cout << "quaternion from matrix = " << q.coeffs().transpose() << endl;

  v_rotated = q *v; // 这里是运算符重载的乘法【四元数乘以三维向量】, q只能放在前面
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;  // 这里是四元数乘法




}