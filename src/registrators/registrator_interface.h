#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace l2l_calib{
namespace registrator{

enum RegistratorType{
  kNoType, kIcpPM, kNdtWithGicp, kKfpcsPcl, kTypeCount
};

struct InlierPointPairs{
  Eigen::Matrix<float, Eigen::Dynamic, 3> ref_points, 
                                          read_points;
  size_t pairs_num;
};

template< typename PointType >
class RegistratorInterface {
public:

  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef pcl::PointCloud<PointType> PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

  RegistratorInterface() = default;
  ~RegistratorInterface() = default;

  virtual void
  setInputSource (const PointCloudSourcePtr &cloud) = 0;
  virtual void
  setInputTarget (const PointCloudTargetPtr &cloud) = 0;
  virtual bool 
  align (const Eigen::Matrix4f& guess, Eigen::Matrix4f& result) = 0;
  virtual
  double getFitnessScore() { return final_score_; }
  virtual
  InlierPointPairs getInlierPointPairs() { return point_pairs_; }

protected:
  double final_score_;

  RegistratorType type_ = kNoType;
  InlierPointPairs point_pairs_;
};

}
}