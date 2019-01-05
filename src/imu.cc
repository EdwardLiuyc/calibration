#include "imu.h"
#include "msg_conversion.h"

namespace l2l_calib {

Imu::Imu(const Pose& tf_pose)
  : last_pose_(Pose::Identity())
  , tf_pose_(tf_pose)
  , bias_(gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0))
  , imu_current_estimate_(bias_,
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(),
      gtsam::Matrix3::Zero(), gtsam::Matrix3::Zero(), gtsam::Matrix::Zero(6,6))
{}

Imu::~Imu() {}

void Imu::Initialise( ros::NodeHandle& nh, std::string topic_name )
{
  subscriber_ = nh.subscribe(topic_name, 10, &Imu::Callback, this );
}

void Imu::Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if( conversion::ToLocalTime(msg->header.stamp) <= newest_time_stamp_ ) {
    PRINT_ERROR("Wrong time in imu msg.");
    return;
  }

  common::MutexLocker locker(&mutex_);
  newest_time_stamp_ = conversion::ToLocalTime(msg->header.stamp);
  imu_msgs_.push(sensors::ToLocalImu(*msg));
}

bool Imu::GetIntergratedPose( const SimpleTime& target_time,
  Pose& pose, Velocity3& velocity )
{
  if( target_time > newest_time_stamp_ ) {
    PRINT_WARNING("please wait for new imu data.");
    pose = last_pose_;
    return false;
  }

  std::vector<LocalImuMsg> used_imus;
  while( imu_msgs_.front().header.stamp <= target_time ) {
    used_imus.push_back(imu_msgs_.front());
    imu_msgs_.pop();
  }
  for( auto& imu : used_imus ) {
    gtsam::Vector3 acc(imu.linear_acceleration.x, 
      imu.linear_acceleration.y, imu.linear_acceleration.z);
    gtsam::Vector3 angle_velocity(imu.angular_velocity.x, 
      imu.angular_velocity.y, imu.angular_velocity.z);
    imu_current_estimate_.integrateMeasurement( acc, angle_velocity, 0.01 );
  }
  gtsam::Rot3 last_rotation = imu_current_estimate_.deltaRij();
  gtsam::Vector3 last_position = imu_current_estimate_.deltaPij();
  
  Pose target_time_pose = Pose::Identity();
  target_time_pose.block(0,3,3,1) = last_position.cast<float>();
  target_time_pose.block(0,0,3,3) = last_rotation.matrix().cast<float>();

  pose = last_pose_ = target_time_pose;
  return true;
}

} // namespace l2l_calib