#ifndef CALIB_IMU_H_
#define CALIB_IMU_H_

#include "common.h"
#include "sensors.h"
#include "ros/ros.h"
#include <queue>

// gtsam
// used for imu preintergration
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

namespace l2l_calib {

class Imu {
public:
  using LocalImuMsg = l2l_calib::sensors::ImuMsg;
  using Pose = Eigen::Matrix4f;
  using Velocity3 = Eigen::Vector3f;

  Imu(const Pose& tf_pose);
  ~Imu();

  /// @brief init the ros topic and subscriber
  void Initialise( ros::NodeHandle& nh, std::string topic_name );
  /*
   * @brief return a pre-intergrated pose on target time
   * @param time : target time 
   * @param pose: output the pose on target time
   * @param velocity: output the velocity on target time
   * @return true if succeed otherwise false
  */
  bool GetIntergratedPose( const SimpleTime& time, 
    Pose& pose, Velocity3& velocity );

  inline 
  Pose GetTfPose() { return tf_pose_; }


protected:
  void Callback(const sensor_msgs::Imu::ConstPtr& msg);

private:
  common::Mutex mutex_;

  std::queue<LocalImuMsg> imu_msgs_;

  ros::Subscriber subscriber_;

  SimpleTime newest_time_stamp_;

  Pose last_pose_;
  Pose tf_pose_;

   // imu preintergration
  gtsam::imuBias::ConstantBias bias_;
  gtsam::CombinedImuFactor::CombinedPreintegratedMeasurements 
    imu_current_estimate_;
};

} // namespace l2l_calib

#endif