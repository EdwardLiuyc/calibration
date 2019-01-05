#ifndef L2L_CALIB_LIDAR_H_
#define L2L_CALIB_LIDAR_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "msg_conversion.h"

#include "common.h"

namespace l2l_calib {

class Lidar {
public:
  using PointType = pcl::PointXYZI;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZI>;
  using PointCloudPtr = PointCloudType::Ptr;

  Lidar() 
    : pose_( Eigen::Matrix4f::Identity() )
  {}

  Lidar( const Eigen::Matrix4f& pose )
    : pose_(pose)
  {}

  ~Lidar() = default;

  void Initialise( ros::NodeHandle& nh, std::string topic_name );

  inline void 
  AddCloud( const PointCloudPtr& cloud ) {
    if( cloud && !cloud->empty() ) {
      clouds_.push_back( cloud );
    } else {
      LOG(ERROR) << "nullptr or empty cloud" << std::endl;
    }
  }

  PointCloudPtr GetNewCloud();
  PointCloudPtr GetNewCloud( 
    const conversion::PclTimeStamp& time_stamp );

  inline
  bool GotFirstData() { return got_first_data_; }

  inline
  Eigen::Matrix4f GetPose() { return pose_; }
  inline 
  void SetPose( const Eigen::Matrix4f& pose ){ pose_ = pose; }

  inline
  void FinishCalibration() { calibration_finished_ = true; }
  inline
  bool FinishedCalibration() { return calibration_finished_; }

  std::string PoseDebugString();

protected:
  void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
  Eigen::Matrix4f pose_;
  std::vector<PointCloudPtr> clouds_;
  
  bool got_first_data_ = false;
  bool calibration_finished_ = false;
  
  ros::Subscriber subscriber_;
};

}

#endif