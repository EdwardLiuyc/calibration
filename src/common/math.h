#ifndef L2L_CALIB_COMMON_MATH_H_
#define L2L_CALIB_COMMON_MATH_H_

#include <cmath>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Eigen"

namespace Eigen {
  template <typename Scalar>
  using Vector6 = Matrix<Scalar, 6, 1>;
}

namespace l2l_calib {
namespace common {

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  while (difference > M_PI) {
    difference -= T(2. * M_PI);
  }
  while (difference < -M_PI) {
    difference += T(2. * M_PI);
  }
  return difference;
}

template <typename T>
Eigen::Matrix<T,3,1> RotationMatrixToEulerAngles(const Eigen::Matrix<T,3,3> &R)
{
  Eigen::Matrix3d double_R = R.template cast<double>();

  double sy = std::sqrt(double_R(0,0)*double_R(0,0)+double_R(1,0)*double_R(1,0) );
  bool singular = sy < 1e-6;

  double x, y, z;
  if (!singular) {
    x = std::atan2(double_R(2,1),double_R(2,2));
    y = std::atan2(-double_R(2,0), sy);
    z = std::atan2(double_R(1,0), double_R(0,0));
  } else {
    x = std::atan2(-double_R(1,2), double_R(1,1));
    y = std::atan2(-double_R(2,0), sy);
    z = 0;
  }
  return Eigen::Vector3d(x,y,z).template cast<T>();
}

template <typename T>
Eigen::Matrix<T,3,3> EulerAnglesToRotationMatrix(
  const Eigen::Matrix<T,3,1> &theta)
{
  // Calculate rotation about x axis
  Eigen::Matrix<double,3,3> R_x, R_y, R_z;
  Eigen::Vector3d double_theta = theta.template cast<double>();
  R_x <<
  1, 0, 0,
  0, std::cos(double_theta[0]), -std::sin(double_theta[0]),
  0, std::sin(double_theta[0]), std::cos(double_theta[0]);
    
  // Calculate rotation about y axis
  R_y <<
  std::cos(double_theta[1]), 0, std::sin(double_theta[1]),
  0, 1, 0,
  -std::sin(double_theta[1]), 0, std::cos(double_theta[1]);
    
  // Calculate rotation about z axis
  R_z <<
  std::cos(double_theta[2]), -std::sin(double_theta[2]), 0,
  std::sin(double_theta[2]), std::cos(double_theta[2]), 0,
  0, 0, 1;

  return (R_z*R_y*R_x).template cast<T>();
}

template <typename T>
Eigen::Quaternion<T> EulerAnglesToQuaternion(
  const Eigen::Matrix<T,3,1> &theta ) 
{
  auto R = EulerAnglesToRotationMatrix(theta);
  return Eigen::Quaternion<T>( R );
}

template <typename T>
Eigen::Vector6<T> TransformToVector6( const Eigen::Matrix<T,4,4>& t )
{
  Eigen::Vector6<T> ret;
  ret.topRows(3) = t.block(0,3,3,1);
  ret.bottomRows(3) = RotationMatrixToEulerAngles(
    Eigen::Matrix<T,3,3>(t.block(0,0,3,3)) );

  return ret;
}

// template <typename T>
// Eigen::Matrix<T,4,4> Vector6ToTransform( const Eigen::Vector6<T>& v ) 
// {
//   Eigen::Matrix<T,4,4> ret = Eigen::Matrix<T,4,4>::Identity();
//   ret.template block<3,3>(0,0) = EulerAnglesToRotationMatrix(
//     Eigen::Matrix<T,3,1>(v.bottomRows(3)));
//   ret.block<3,1>(0,3) = v.topRows(3);

//   return ret;
// }

template <typename T>
Eigen::Matrix<T,4,4> Vector6ToTransform( const Eigen::Vector6<T>& v ) 
{
  Eigen::Matrix<T,4,4> ret = Eigen::Matrix<T,4,4>::Identity();

  Eigen::AngleAxis<T> yaw_angle(v[5], Eigen::Matrix<T,3,1>::UnitZ());
  Eigen::AngleAxis<T> pitch_angle(v[4], Eigen::Matrix<T,3,1>::UnitY());
  Eigen::AngleAxis<T> roll_angle(v[3], Eigen::Matrix<T,3,1>::UnitX());
  Eigen::Quaternion<T> q = yaw_angle * pitch_angle * roll_angle;

  ret.template block<3,1>(0,3) = v.topRows(3);
  ret.template block<3,3>(0,0) = q.toRotationMatrix();

  return ret;
}

}  // namespace common
}  // namespace l2l_calib

#endif // L2L_CALIB_COMMON_MATH_H_

