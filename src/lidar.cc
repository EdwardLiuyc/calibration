#include "lidar.h"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace l2l_calib {

void Lidar::Initialise( 
  ros::NodeHandle& nh, std::string topic_name )
{
  subscriber_ = nh.subscribe(topic_name, 10, &Lidar::Callback, this );
}

void Lidar::Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if( !got_first_data_ ) {
    got_first_data_ = true;
  }

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL( *msg, pcl_pc2 );
  PointCloudPtr incoming_cloud( new PointCloudType );
  pcl::fromPCLPointCloud2( pcl_pc2, *incoming_cloud );

  AddCloud( incoming_cloud );
}

Lidar::PointCloudPtr Lidar::GetNewCloud()
{
  if( clouds_.empty() ) {
    return nullptr;
  }

  auto ret = clouds_[0];
  clouds_.erase( clouds_.begin() );
  return ret;
}

Lidar::PointCloudPtr Lidar::GetNewCloud(
  const conversion::PclTimeStamp& target_time_stamp )
{
  if( clouds_.empty() ) {
    return nullptr;
  }
  
  Lidar::PointCloudPtr ret = nullptr;
  uint64_t tolerant = 80000;
  while( !clouds_.empty() ) {
    auto cloud_time = clouds_.front()->header.stamp;
    if( cloud_time >= target_time_stamp && 
      cloud_time - target_time_stamp <= tolerant ) {
      ret = clouds_[0];
    } else {
      auto delta = target_time_stamp - cloud_time;
      if( delta <= tolerant ) {
        ret = clouds_[0];
      }
    }

    clouds_.erase( clouds_.begin() );
    if( ret )
      break;
  }
  return ret;
}

std::string Lidar::PoseDebugString()
{
  std::ostringstream out;
  out << "Pose Matrix : \n" << pose_ << std::endl;
  out << "translation : \n" << "  x: " << pose_(0,3)
    << "\n  y: " << pose_(1,3) << "\n  z: " << pose_(2,3) << std::endl;
  Eigen::Quaternionf q( Eigen::Matrix3f(pose_.block(0,0,3,3)) );
  out << "quaternion  : \n" << "  x: " << q.x()
    << "\n  y: " << q.y() << "\n  z: " << q.z()
    << "\n  w: " << q.w() << std::endl;

  return out.str();
}

}