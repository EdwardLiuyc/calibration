#ifndef ICP_POINTMATCHER_H_
#define ICP_POINTMATCHER_H_

#include "msg_conversion.h"
#include "registrator_interface.h"
#include <glog/logging.h>
#include <fstream>

namespace l2l_calib{
namespace registrator{

template <typename PointType>
class IcpUsingPointMatcher : 
  public RegistratorInterface<PointType> {
public:

  typedef PointMatcher<float> PM;

  using typename 
    RegistratorInterface<PointType>::PointCloudSourcePtr;
  using typename 
    RegistratorInterface<PointType>::PointCloudTargetPtr;

  IcpUsingPointMatcher( const std::string& ymal_file = "" )
    : RegistratorInterface<PointType>()
    , reference_cloud_( new PM::DataPoints )
    , reading_cloud_( new PM::DataPoints )
    {
      RegistratorInterface<PointType>::type_ = kIcpPM;
      loadConfig(ymal_file);
    }
  ~IcpUsingPointMatcher() {}

  inline
  void setInputSource (const PointCloudSourcePtr &cloud) override
  {
    if( !cloud || cloud->empty() ) {
      PRINT_ERROR("Empty cloud.");
      return;
    }
    *reading_cloud_ = 
      conversion::pclPointCloudToLibPointMatcherPoints<PointType>( cloud );

    CHECK(reading_cloud_->getNbPoints() == cloud->points.size());
  }

  inline 
  void setInputTarget (const PointCloudTargetPtr &cloud) override
  {
    if( !cloud || cloud->empty() ) {
      PRINT_ERROR("Empty cloud.");
      return;
    }
    *reference_cloud_ = 
      conversion::pclPointCloudToLibPointMatcherPoints<PointType>( cloud );

    CHECK(reference_cloud_->getNbPoints() == cloud->points.size());
  }

  inline 
  bool align (const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) override
  {
    // **** compute the transform ****
    result = pm_icp_.compute( *reading_cloud_, *reference_cloud_, guess );

    // **** compute the final score **** 
    PM::DataPoints data_out( *reading_cloud_ );
	  pm_icp_.transformations.apply( data_out, result );
    pm_icp_.matcher->init( *reference_cloud_ );

    // extract closest points
    PM::Matches matches = pm_icp_.matcher->findClosests(data_out);

    // weight paired points
    const PM::OutlierWeights outlier_weights = 
      pm_icp_.outlierFilters.compute(data_out, *reference_cloud_, matches);
	
    // generate tuples of matched points and remove pairs with zero weight
    const PM::ErrorMinimizer::ErrorElements matched_points( 
      data_out, *reference_cloud_, outlier_weights, matches);

    // extract relevant information for convenience
    const int dim = matched_points.reading.getEuclideanDim();
    const int matcher_points_num = matched_points.reading.getNbPoints(); 
    const PM::Matrix matched_read = 
      matched_points.reading.features.topRows(dim);
    const PM::Matrix matched_ref = 
      matched_points.reference.features.topRows(dim);

    CHECK(matched_read.rows() == matched_ref.rows() 
      && matched_read.cols() == matched_ref.cols()
      && matched_read.rows() == 3);

    RegistratorInterface<PointType>::point_pairs_.ref_points
      = matched_ref.transpose().cast<float>();
    RegistratorInterface<PointType>::point_pairs_.read_points
      = matched_read.transpose().cast<float>();
	  RegistratorInterface<PointType>::point_pairs_.pairs_num
      = matched_read.cols();

    // compute mean distance
    const PM::Matrix dist = (matched_read - matched_ref).colwise().norm(); 
    // replace that by squaredNorm() to save computation time
    const float mean_dist = dist.sum()/(float)matcher_points_num;
    // cout << "Robust mean distance: " << mean_dist << " m" << endl;
    RegistratorInterface<PointType>::final_score_ = std::exp( -mean_dist );

    if( RegistratorInterface<PointType>::final_score_ < 0.6 ) {
      return false;
    }
    return true;
  }

protected:
  void loadDefaultConfig();
  void loadConfig( const std::string& yaml_filename );

private:

  std::shared_ptr<PM::DataPoints> reference_cloud_;
  std::shared_ptr<PM::DataPoints> reading_cloud_; 

  PM::ICP pm_icp_;
};

}  // namespace registrator
}  // namespace l2l_calib

#include "icp_pointmatcher_impl.hpp"

#endif