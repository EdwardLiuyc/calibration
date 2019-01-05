#ifndef REGISTRATORS_NDT_GICP_H_
#define REGISTRATORS_NDT_GICP_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "registrator_interface.h"

#ifdef NDT_USE_GPU
#include "ndt_gpu/NormalDistributionsTransform.h"
#else
#include <pcl/registration/ndt.h>
#endif

#include "libicp/icpPointToPlane.h"
#include "libicp/icpPointToPoint.h"

#include <vector>
#include <cmath>
#include "common/macro_defines.h"

namespace l2l_calib{
namespace registrator{
  
template <typename PointType>
class RegistrationWithNDTandGICP
 : public RegistratorInterface<PointType>
{
  using typename 
    RegistratorInterface<PointType>::PointCloudSource;
  using typename 
    RegistratorInterface<PointType>::PointCloudTarget;
  using typename 
    RegistratorInterface<PointType>::PointCloudSourcePtr;
  using typename 
    RegistratorInterface<PointType>::PointCloudTargetPtr;
  
public:
  
  RegistrationWithNDTandGICP( bool using_voxel_filter = true, 
                              double voxel_resolution = 0.2)
    : input_cloud_( new PointCloudSource )
    , target_cloud_( new PointCloudTarget )
    , down_sampled_input_cloud_( new PointCloudSource )
    , down_sampled_target_cloud_( new PointCloudTarget )
    , voxel_resolution_( voxel_resolution )
    , using_voxel_filter_( using_voxel_filter )
  {
    RegistratorInterface<PointType>::type_ = kNdtWithGicp;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt_.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt_.setStepSize( 0.1 );
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt_.setResolution(1.);
    //设置匹配迭代的最大次数
    ndt_.setMaximumIterations(35);
    
    gicp_.setRotationEpsilon(1e-3);
    gicp_.setMaximumIterations(35);
    
    approximate_voxel_filter_.setLeafSize(voxel_resolution_, 
      voxel_resolution_, voxel_resolution_);
  }
  
  ~RegistrationWithNDTandGICP() = default;
  
  typedef boost::shared_ptr< RegistrationWithNDTandGICP<PointType> > 
    Ptr;
  typedef boost::shared_ptr< const RegistrationWithNDTandGICP<PointType> > 
    ConstPtr;
  
  inline void
  setInputSource (const PointCloudSourcePtr &cloud) override
  {
    if (cloud->points.empty ()) {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    *input_cloud_ = *cloud;
  }
  
  inline void
  setInputTarget (const PointCloudTargetPtr &cloud) override
  {
    if (cloud->points.empty ()) {
      PCL_ERROR ("Invalid or empty point cloud dataset given!\n");
      return;
    }
    *target_cloud_ = *cloud;
  }
  
  bool align (const Eigen::Matrix4f& guess, Eigen::Matrix4f& result)
  {
    if( using_voxel_filter_ )
    {
      approximate_voxel_filter_.setInputCloud( input_cloud_ );
      approximate_voxel_filter_.filter(*down_sampled_input_cloud_);
      
      approximate_voxel_filter_.setInputCloud( target_cloud_ );
      approximate_voxel_filter_.filter(*down_sampled_target_cloud_);
    }
    else
    {
      *down_sampled_input_cloud_ = *input_cloud_;
      *down_sampled_target_cloud_ = *target_cloud_;
    }
    
    PointCloudSourcePtr output_cloud(new PointCloudSource);
    
    Eigen::Matrix4f ndt_guess = guess;
    double ndt_score = 0.9;
    if( use_ndt_ )
    {
      ndt_.setInputSource(down_sampled_input_cloud_);
      ndt_.setInputTarget(down_sampled_target_cloud_);
      
      #ifdef NDT_USE_GPU
      ndt_.align( guess );
      #else
      ndt_.align(*output_cloud, guess);
      #endif
      
      ndt_score = ndt_.getFitnessScore();
      
      #ifdef NDT_USE_GPU
      ndt_score /= 20;
      #endif

      ndt_guess = ndt_.getFinalTransformation();
    }
    
    double icp_score = 10.;
    Eigen::Matrix4f final_guess;
    if( ndt_score <= 1. )
    {  
      icp_score = ndt_score;
      
      gicp_.setInputSource( down_sampled_input_cloud_ );
      gicp_.setInputTarget( down_sampled_target_cloud_ );
      gicp_.align(*output_cloud, ndt_guess);
      
      icp_score = gicp_.getFitnessScore();
      final_guess = gicp_.getFinalTransformation();
      
      // final_guess(2,3) = 0.;     

      RegistratorInterface<PointType>::final_score_ = std::exp( -icp_score );
      result = final_guess;
      return true;
    }
    else 
    {
      result = guess;
      final_score_ = std::exp( -icp_score );
      return false;
    }
    
    return true;
  }
  
  inline 
  double getFitnessScore() { return final_score_; }
  
  inline
  void enableNdt( bool use_ndt ){ use_ndt_ = use_ndt; }
  
private:
#ifdef NDT_USE_GPU
  gpu::GNormalDistributionsTransform ndt_;
#else 
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;
#endif
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_;
  
  PointCloudSourcePtr input_cloud_;
  PointCloudTargetPtr target_cloud_;
  
  PointCloudSourcePtr down_sampled_input_cloud_;
  PointCloudTargetPtr down_sampled_target_cloud_;
  
  pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter_;
  double voxel_resolution_;
  bool using_voxel_filter_;
  bool use_ndt_ = true;
  
  double final_score_ = 0.;
};
}  // namespace registrator
}  // namespace l2l_calib

#endif // REGISTRATORS_NDT_GICP_H_