#ifndef L2L_CALIB_MSG_COMVERSIONS_H_
#define L2L_CALIB_MSG_COMVERSIONS_H_

#include "common.h"
#include "ros/time.h"
#include "libpointmatcher/pointmatcher/PointMatcher.h"

namespace l2l_calib {
namespace conversion {

typedef uint64_t PclTimeStamp;
SimpleTime ToLocalTime( const ros::Time& time );
SimpleTime ToLocalTime( const PclTimeStamp& time );

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
template <typename PointT>
DP pclPointCloudToLibPointMatcherPoints( 
  const typename pcl::PointCloud<PointT>::Ptr& pcl_point_cloud )
{
  if( !pcl_point_cloud || pcl_point_cloud->empty() ){
    return DP();
  }

  DP::Labels labels;
  labels.push_back(DP::Label("x", 1));
  labels.push_back(DP::Label("y", 1));
  labels.push_back(DP::Label("z", 1));
  labels.push_back(DP::Label("pad", 1));

  const size_t point_count(pcl_point_cloud->points.size());
  PM::Matrix inner_cloud(4, point_count);
  int index = 0;
  for (size_t i = 0; i < point_count; ++i)
  {
    if (!std::isnan(pcl_point_cloud->points[i].x) && 
      !std::isnan(pcl_point_cloud->points[i].y) &&
      !std::isnan(pcl_point_cloud->points[i].z) ) {
      inner_cloud(0, index) = pcl_point_cloud->points[i].x;
      inner_cloud(1, index) = pcl_point_cloud->points[i].y;
      inner_cloud(2, index) = pcl_point_cloud->points[i].z;
      inner_cloud(3, index) = 1;
      ++index;
    }
  }

  DP d(inner_cloud.leftCols(index), labels);
  return std::move( d );
}
  
} // namespace conversion
} // namespace l2l_calib

#endif