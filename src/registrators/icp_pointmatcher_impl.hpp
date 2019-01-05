#ifndef ICP_POINTMATCHER_IMPL_HPP_
#define ICP_POINTMATCHER_IMPL_HPP_

namespace l2l_calib{
namespace registrator{

template<typename PointType>
void IcpUsingPointMatcher<PointType>::
  loadConfig( const std::string& yaml_filename )
{
  if( yaml_filename.empty() ) {
    loadDefaultConfig();
    return;
  }

  // load YAML config
  std::ifstream ifs(yaml_filename.c_str());
  if (!ifs.good()) {
    PRINT_ERROR_FMT("Cannot open config file: %s", yaml_filename.c_str());
    exit(1);
  }
  pm_icp_.loadFromYaml(ifs);
}

template<typename PointType>
void IcpUsingPointMatcher<PointType>::loadDefaultConfig()
{
  // config 
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // Prepare reading filters
  name = "MinDistDataPointsFilter";
  params["minDist"] = "1.";
  std::shared_ptr<PM::DataPointsFilter> minDist_read =
    PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  name = "RandomSamplingDataPointsFilter";
  params["prob"] = "0.05";
  std::shared_ptr<PM::DataPointsFilter> rand_read =
    PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // Prepare reference filters
  name = "MinDistDataPointsFilter";
  params["minDist"] = "1.";
  std::shared_ptr<PM::DataPointsFilter> minDist_ref =
    PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  name = "SamplingSurfaceNormalDataPointsFilter";
  params["knn"] = "7";
  params["samplingMethod"] = "1";
  params["ratio"] = "0.1";
  std::shared_ptr<PM::DataPointsFilter> normal_ref =
    PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // Prepare matching function
  name = "KDTreeMatcher";
  params["knn"] = "1";
  params["epsilon"] = "3.16";
  std::shared_ptr<PM::Matcher> kdtree =
    PM::get().MatcherRegistrar.create(name, params);
  params.clear();

  // Prepare outlier filters
  name = "TrimmedDistOutlierFilter";
  params["ratio"] = "0.7";
  std::shared_ptr<PM::OutlierFilter> trim =
    PM::get().OutlierFilterRegistrar.create(name, params);
  params.clear();

  // Prepare error minimization
  name = "PointToPointErrorMinimizer";
  std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
    PM::get().ErrorMinimizerRegistrar.create(name);

  name = "PointToPlaneErrorMinimizer";
  std::shared_ptr<PM::ErrorMinimizer> pointToPlane =
    PM::get().ErrorMinimizerRegistrar.create(name);

  // Prepare transformation checker filters
  name = "CounterTransformationChecker";
  params["maxIterationCount"] = "150";
  std::shared_ptr<PM::TransformationChecker> maxIter =
    PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "DifferentialTransformationChecker";
  params["minDiffRotErr"] = "0.001";
  params["minDiffTransErr"] = "0.01";
  params["smoothLength"] = "4";
  std::shared_ptr<PM::TransformationChecker> diff =
    PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  std::shared_ptr<PM::Transformation> rigidTrans =
      PM::get().TransformationRegistrar.create("RigidTransformation");
  // Prepare inspector
  std::shared_ptr<PM::Inspector> nullInspect =
    PM::get().InspectorRegistrar.create("NullInspector");

  // data filters
  pm_icp_.readingDataPointsFilters.push_back(minDist_read);
  pm_icp_.readingDataPointsFilters.push_back(rand_read);
  pm_icp_.referenceDataPointsFilters.push_back(minDist_ref);
  pm_icp_.referenceDataPointsFilters.push_back(normal_ref);
  // matcher
  pm_icp_.matcher = kdtree;
  // outlier filter
  pm_icp_.outlierFilters.push_back(trim);
  // error minimizer
  pm_icp_.errorMinimizer = pointToPlane;
  // checker
  pm_icp_.transformationCheckers.push_back(maxIter);
  pm_icp_.transformationCheckers.push_back(diff);

  pm_icp_.inspector = nullInspect;
  // result transform
  pm_icp_.transformations.push_back(rigidTrans);
}


}
}




#endif // ICP_POINTMATCHER_IMPL_HPP_