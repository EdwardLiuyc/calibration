#include "sensors.h"
#include "msg_conversion.h"

using namespace l2l_calib::conversion;

namespace l2l_calib {
namespace sensors {

inline
Header ToLocalHeader( const std_msgs::Header& header )
{
  Header local_header;
  
  local_header.seq = header.seq;
  local_header.frame_id = header.frame_id;
  local_header.stamp = ToLocalTime(header.stamp);
  
  return local_header;
}

ImuMsg ToLocalImu( const sensor_msgs::Imu& msg )
{
  ImuMsg local_imu;
  
  local_imu.header = ToLocalHeader( msg.header );
  
  local_imu.angular_velocity.x = msg.angular_velocity.x;
  local_imu.angular_velocity.y = msg.angular_velocity.y;
  local_imu.angular_velocity.z = msg.angular_velocity.z;
  
  local_imu.linear_acceleration.x = msg.linear_acceleration.x;
  local_imu.linear_acceleration.y = msg.linear_acceleration.y;
  local_imu.linear_acceleration.z = msg.linear_acceleration.z;
  
  local_imu.orientation.x = msg.orientation.x;
  local_imu.orientation.y = msg.orientation.y;
  local_imu.orientation.z = msg.orientation.z;
  local_imu.orientation.w = msg.orientation.w;
  
  for( int i = 0; i < 9; ++i )
  {
    local_imu.angular_velocity_covariance[i] = 
      msg.angular_velocity_covariance[i];
    local_imu.linear_acceleration_covariance[i] = 
      msg.linear_acceleration_covariance[i];
    local_imu.orientation_covariance[i] = msg.orientation_covariance[i];
  }
  
  return local_imu;
}

} // namespace sensors
} // namespace l2l_calib