#ifndef L2L_CALIB_COST_FUNCTION_H_
#define L2L_CALIB_COST_FUNCTION_H_

#include "common/math.h"
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/numeric_diff_cost_function.h"
#include "ceres/problem.h"

namespace l2l_calib {

using common::Vector6ToTransform;
using common::TransformToVector6;

class ModelImuToLidarCostFunctor {
public:
  ModelImuToLidarCostFunctor(const Eigen::Matrix4f& lidar_pose,
                              const Eigen::Matrix4f& imu_pose)
    : lidar_pose_(lidar_pose), imu_pose_(imu_pose) {}

  template <typename T>
  bool operator()(const T* const tf_lidar_base_link, 
                  const T* const tf_imu_bask_link, T* residuals_ptr) const {
    Eigen::Map<const Eigen::Vector6<T>> lidar_base_vector(tf_lidar_base_link);
    Eigen::Map<const Eigen::Vector6<T>> imu_base_vector(imu_base_vector);
    
    Eigen::Matrix<T,4,4> tf_l_b_matrix = Vector6ToTransform(
      Eigen::Vector6<T>(lidar_base_vector));
    Eigen::Matrix<T,4,4> tf_i_b_matrix = Vector6ToTransform(
      Eigen::Vector6<T>(imu_base_vector));

    Eigen::Matrix<T,4,4> pose_estimate_l = lidar_pose_.template cast<T>() * 
      tf_l_b_matrix.conjugate();
    Eigen::Matrix<T,4,4> pose_estimate_i = imu_pose_.template cast<T>() * 
      tf_i_b_matrix.conjugate();

    Eigen::Quaternion<T> delta_q =
        Eigen::Quaternion<T>(pose_estimate_l.template block<3,3>(0,0)) * 
        Eigen::Quaternion<T>(pose_estimate_i.template block<3,3>(0,0)).conjugate();

    Eigen::Map<Eigen::Vector6<T>> residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        pose_estimate_l.template block<3,1>(0,3) - pose_estimate_i.template block<3,1>(0,3);
    residuals.template block<3, 1>(3, 0) = delta_q.vec();

    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix4f& lidar_pose,
      const Eigen::Matrix4f& imu_pose) {
    return new ceres::AutoDiffCostFunction<ModelImuToLidarCostFunctor, 6, 6, 6>(
             new ModelImuToLidarCostFunctor(lidar_pose, imu_pose));
  }

private:
  const Eigen::Matrix4f lidar_pose_;
  const Eigen::Matrix4f imu_pose_;
};


class ImuToLidarCostFunctor {
public:
  ImuToLidarCostFunctor(const Eigen::Matrix4f& lidar_pose,
                        const Eigen::Matrix4f& imu_pose)
    : lidar_pose_(lidar_pose), imu_pose_(imu_pose) {}

  bool operator()(const double* const tf_lidar_base_link, 
                  const double* const tf_imu_bask_link, 
                  double* residuals_ptr) const {
    
    Eigen::Map<const Eigen::Vector6<double>> lidar_base_vector(
      tf_lidar_base_link);
    Eigen::Map<const Eigen::Vector6<double>> imu_base_vector(
      imu_base_vector);
    
    Eigen::Matrix4d tf_l_b_matrix = Vector6ToTransform(
      Eigen::Vector6<double>(lidar_base_vector));
    Eigen::Matrix4d tf_i_b_matrix = Vector6ToTransform(
      Eigen::Vector6<double>(imu_base_vector));

    Eigen::Matrix4d pose_estimate_l = lidar_pose_.cast<double>() * 
      tf_l_b_matrix.conjugate();
    Eigen::Matrix4d pose_estimate_i = imu_pose_.cast<double>() * 
      tf_i_b_matrix.conjugate();
    Eigen::Vector6<double> residuals_vector = 
      TransformToVector6(pose_estimate_l) - 
      TransformToVector6(pose_estimate_i);

    Eigen::Map<Eigen::Vector6<double>> residuals(residuals_ptr);
    residuals = residuals_vector;
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Matrix4f& lidar_pose,
      const Eigen::Matrix4f& imu_pose) {
    return new ceres::NumericDiffCostFunction<ImuToLidarCostFunctor, 
                ceres::CENTRAL, 6, 6, 6>(
                  new ImuToLidarCostFunctor(lidar_pose, imu_pose));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix4f lidar_pose_;
  const Eigen::Matrix4f imu_pose_;
};

} // namespace l2l_calib

#endif // L2L_CALIB_COST_FUNCTION_H_