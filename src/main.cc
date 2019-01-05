#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "calibrator.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/console/parse.h>
#include "options.h"
#include "common/math.h"

using namespace l2l_calib::common;

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "l2l_calib_node");
  ros::NodeHandle n;

  std::string cfg_filename = "";
  pcl::console::parse_argument(argc, argv, "-cfg", cfg_filename);
  if( cfg_filename.empty() ) {
    std::cout << "You should use the node this way:\n\n"
      << "    l2l_calib_node -cfg [config_file_name.xml]\n" << std::endl;
    return -1;
  }

  std::vector<l2l_calib::DeviceCoordOption> options;
  if( l2l_calib::ReadOptions(cfg_filename, options) < 0 || options.empty()) {
    return -1;
  }

  l2l_calib::CalibratorSettings settings;
  auto& lidar_settings = settings.lidar_settings;
  auto& imu_settings = settings.imu_settings;
  for( auto& option : options ) {
    l2l_calib::TfSetting setting;
    setting.topic_name = option.topic_name;
    setting.translation << option.translation.x,
      option.translation.y, option.translation.z;
    auto& q = option.quaternion;
    setting.quaternion = Eigen::Quaternionf(
      q.w, q.x, q.y, q.z );

    switch(option.type) {
      case l2l_calib::kLidar:
        lidar_settings.push_back(setting);
        break;
      case l2l_calib::kImu:
        imu_settings.push_back(setting);
        break;
      default:
        break;
    }
  }

  ros::Publisher fused_cloud_publisher 
    = n.advertise<sensor_msgs::PointCloud2> ("/map", 1);
  l2l_calib::Calibrator::ShowFusedCloudFunc show_function = 
    [&fused_cloud_publisher]( const l2l_calib::Lidar::PointCloudPtr& cloud )
    {
      sensor_msgs::PointCloud2 map_pointcloud;
      pcl::toROSMsg(*cloud, map_pointcloud);

      map_pointcloud.header.frame_id = "/fused_cloud";
      map_pointcloud.header.stamp = ros::Time::now();
      fused_cloud_publisher.publish(map_pointcloud);
    };
  
  l2l_calib::Calibrator calibrator;
  calibrator.SetShowFunction( show_function );
  calibrator.Initialise( n, settings );

  ros::Rate loop_rate(1000);
  while( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  calibrator.EndAllComputation();
  
  return 0;
}