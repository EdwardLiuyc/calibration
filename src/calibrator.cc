#include "calibrator.h"
#include "sensor_msgs/PointCloud2.h"
#include "registrators/icp_pointmatcher.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "cost_function.h"
#include "ceres/local_parameterization.h"

using l2l_calib::common::RotationMatrixToEulerAngles;
using l2l_calib::common::EulerAnglesToRotationMatrix;

namespace l2l_calib {

void DownSamplePointcloud( Lidar::PointCloudPtr& source, 
  Lidar::PointCloudPtr& output)
{
  auto get_range_distance = 
    []( const Lidar::PointType& a ){ 
      return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z); };

  Lidar::PointCloudPtr tmp_cloud( new Lidar::PointCloudType );
  for( auto& point : source->points ) {
    if( get_range_distance(point) <= 80.
      && get_range_distance(point) >= 2. ) {
      tmp_cloud->push_back(point);
    }
  }

  pcl::StatisticalOutlierRemoval<Lidar::PointType> sor;
  sor.setInputCloud ( tmp_cloud );
  sor.setMeanK ( 30 );                               
  sor.setStddevMulThresh ( 1. );                  
  sor.filter (*output); 
}

Eigen::Matrix4f AverageTransforms( 
  const std::vector<Eigen::Matrix4f>& transforms )
{
  CHECK( !transforms.empty() );
  Eigen::Vector3f angles( 0, 0, 0 );
  Eigen::Vector3f translation( 0, 0, 0 );
  for( auto transform : transforms ) {
    translation += transform.block(0,3,3,1);
    angles += RotationMatrixToEulerAngles(
      Eigen::Matrix3f( transform.block(0,0,3,3) ));
  }
  translation /= (float)transforms.size();
  angles /= (float)transforms.size();
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block(0,0,3,3) = EulerAnglesToRotationMatrix(angles);
  result.block(0,3,3,1) = translation;

  return result;
}

void Calibrator::Initialise( ros::NodeHandle& nh,
  const CalibratorSettings& settings )
{
  auto& lidar_settings = settings.lidar_settings;
  auto& imu_settings = settings.imu_settings;

  // settings for lidar
  CHECK_GE( lidar_settings.size(), 1 );
  CHECK_LE( imu_settings.size(), 1 );
  lidars_.resize( lidar_settings.size() );
  for( size_t i = 0; i < lidar_settings.size(); ++i ) {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block(0,0,3,3) = 
      Eigen::Matrix3f( lidar_settings[i].quaternion.toRotationMatrix() );
    pose.block(0,3,3,1) = lidar_settings[i].translation;
    
    lidars_[i] = std::make_shared<Lidar>( pose );
    lidars_[i]->Initialise( nh, lidar_settings[i].topic_name );
  }
  scan_matcher_ = std::make_shared<
    registrator::IcpUsingPointMatcher<Lidar::PointType>>();
  scan_match_thread_ = std::make_shared<std::thread>(
     std::bind(&Calibrator::ScanMatching, this ) );

  if( !imu_settings.empty() ) {
    auto imus_count = imu_settings.size();
    imus_.resize(imus_count);
    imus_motion_data_.resize(imus_count);
    for( size_t i = 0; i < imus_count; ++i ) {
      Eigen::Matrix4f tf_pose = Eigen::Matrix4f::Identity();
      tf_pose.block(0,0,3,3) = 
        Eigen::Matrix3f( imu_settings[i].quaternion.toRotationMatrix() );
      tf_pose.block(0,3,3,1) = imu_settings[i].translation;

      imus_[i] = std::make_shared<Imu>( tf_pose );
      imus_[i]->Initialise( nh, imu_settings[i].topic_name );

      imu_calib_thread_ = std::make_shared<std::thread>(
        std::bind(&Calibrator::ImuCalibrating, this ) );
    }
  }
}

void Calibrator::ScanMatching()
{
  scan_match_thread_running_ = true;
  std::vector<Lidar::PointCloudPtr> clouds_from_diff_lidars;
  std::vector<Lidar::PointCloudPtr> transformed_clouds;
  std::vector<std::vector<Eigen::Matrix4f>> history_poses;

  history_poses.resize(lidars_.size());
  clouds_from_diff_lidars.resize(lidars_.size());
  transformed_clouds.resize(lidars_.size());
  for( auto& transformed_cloud : transformed_clouds ) {
    transformed_cloud = boost::make_shared<Lidar::PointCloudType>();
  }

  while( !lidar_calibration_finished_ ) {
    if( force_quit_ ) {
      break;
    }

    // step 1 
    // get time synced data from lidars
    if( !lidars_[0]->GotFirstData() ) {
      SimpleTime::from_sec(0.1).sleep();
      continue;
    }

    bool all_lidar_data_ready = false;
    uint32_t wait_times = 0;
    SimpleTime target_time;
    while( !all_lidar_data_ready && wait_times <= 40 ) {
      size_t data_count = 0;
      clouds_from_diff_lidars[0] = lidars_[0]->GetNewCloud();
      if( clouds_from_diff_lidars[0] ) {
        data_count++;
        auto tmp_time = clouds_from_diff_lidars[0]->header.stamp;
        target_time = conversion::ToLocalTime(tmp_time);
        for( size_t i = 1; i < lidars_.size(); ++i ) {
          clouds_from_diff_lidars[i] = lidars_[i]->GetNewCloud( tmp_time );
          if( clouds_from_diff_lidars[i] ) {
            data_count++;
          }
        }
      }
      if( data_count == lidars_.size() ) {
        all_lidar_data_ready = true;
      } else {
        SimpleTime::from_sec(0.025).sleep();
        wait_times++;
      }
    }

    if( !all_lidar_data_ready ) {
      continue;
    }

    if( !imus_.empty() ) {
      wait_times = 0;
      bool all_imu_data_ready = false;
      std::vector<ImuMotionData> motion_data;
      motion_data.resize(imus_.size());
      while( !all_imu_data_ready && wait_times <= 40 ) {
        size_t data_count = 0;
        for( size_t i = 0; i < imus_.size(); ++i ) {
          auto& imu = imus_.at(i);
          // auto& poses = imu_pre_intergrated_poses_.at(i);
          Imu::Pose pose; 
          Imu::Velocity3 vel;
          if( imu->GetIntergratedPose(target_time, pose, vel) ) {
            data_count++;
            motion_data[i].pose = pose;
            motion_data[i].velocity = vel;
          }
        }
        if( data_count == imus_.size() ) {
          all_imu_data_ready = true;
        } else {
          SimpleTime::from_sec(0.01).sleep();
          wait_times++;
        }
      }
      if( !all_imu_data_ready ) {
        continue;
      }
      {
        common::MutexLocker locker(&mutex_);
        for(size_t i = 0; i < imus_.size(); ++i) {
          imus_motion_data_[i].push( motion_data[i] );
        }
        clouds_for_imu_calib_.push(clouds_from_diff_lidars[0]);
      }
    }
    
    // step 2
    // transformed all data to the same coordinate
    for( size_t i = 0; i < transformed_clouds.size(); ++i ) {
      auto tmp_cloud = boost::make_shared<Lidar::PointCloudType>();
      DownSamplePointcloud( clouds_from_diff_lidars[i], tmp_cloud );
      pcl::transformPointCloud( *tmp_cloud,
        *transformed_clouds[i], lidars_[i]->GetPose() );
    }

    // step 3 
    // scan match 
    scan_matcher_->setInputTarget( transformed_clouds[0] );
    for( size_t i = 1; i < transformed_clouds.size(); ++i ) {
      if( lidars_[i]->FinishedCalibration() ) {
        continue;
      }
      scan_matcher_->setInputSource( transformed_clouds[i] );

      Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
      scan_matcher_->align( guess, result );

      double score = scan_matcher_->getFitnessScore();
      LOG(INFO) << "score = " << score << std::endl;
      if( score > 0.75 ) {
        if( result.block(0,3,3,1).norm() < 0.02
          && (Eigen::Matrix3f(result.block(0,0,3,3))-Eigen::Matrix3f::Identity()).norm() < 0.01 ) {
          history_poses[i].push_back(lidars_[i]->GetPose());
        } else {
          history_poses[i].clear();
        }

        if( history_poses[i].size() >= 3 ) {
          auto calib_pose = AverageTransforms(history_poses[i]);
          lidars_[i]->SetPose(calib_pose);
          lidars_[i]->FinishCalibration();
          PRINT_INFO_FMT("Calibration finished :\n%s", lidars_[i]->PoseDebugString().c_str());
        }

        if( !lidars_[i]->FinishedCalibration() ) {
          lidars_[i]->SetPose( lidars_[i]->GetPose() * result );
          PRINT_INFO("Set new pose");
        }
        
      } else {
        history_poses[i].clear();
      }
    }

    if( show_fused_cloud_function_ ) {
      float intensity = 0.2;
      auto tmp_cloud = boost::make_shared<Lidar::PointCloudType>();
      for( size_t i = 0; i < transformed_clouds.size(); ++i ) {
        pcl::transformPointCloud( *clouds_from_diff_lidars[i],
          *transformed_clouds[i], lidars_[i]->GetPose() );
        for( auto& point : transformed_clouds[i]->points ) {
          point.intensity = intensity + 0.6 * i;
        }
        *tmp_cloud += *transformed_clouds[i];
      }
      show_fused_cloud_function_(tmp_cloud);
    }
  }

  PRINT_INFO("lidar calibration thread exit.");
  scan_match_thread_running_ = false;
}

void Calibrator::ImuCalibrating()
{
  imu_calib_thread_running_ = true;
  auto imu_size = imus_.size();
  auto matcher = std::make_shared<
    registrator::IcpUsingPointMatcher<Lidar::PointType>>();

  auto translation_in_pose = 
    [](const Imu::Pose& pose) -> Eigen::Vector3f {
      return Eigen::Vector3f(pose.block(0,3,3,1));
    };
  auto translation_of_two_pose = 
    [&translation_in_pose](const Imu::Pose& pose_a, const Imu::Pose& pose_b)
      -> float {
      return (translation_in_pose(pose_a)-translation_in_pose(pose_b)).norm();
    };

  bool got_first_data = false;
  Lidar::PointCloudPtr current_cloud, last_cloud;
  std::vector<ImuMotionData> current_imu_motions, last_imu_motions;
  current_imu_motions.resize(imu_size);
  last_imu_motions.resize(imu_size);

  using LidarPose = Imu::Pose;
  std::vector<LidarPose> lidar_poses;
  std::vector<std::vector<Imu::Pose>> imus_poses;
  imus_poses.resize(imu_size);

#define SKIP_LOOP \
  SimpleTime::from_sec(0.01).sleep(); \
  continue;

  while( !force_quit_ ) {
    if( clouds_for_imu_calib_.empty() ) {
      SKIP_LOOP;
    }
    
    // step1 get current neweat data
    {
      common::MutexLocker locker(&mutex_);
      current_cloud = clouds_for_imu_calib_.front();
      auto size = clouds_for_imu_calib_.size();
      clouds_for_imu_calib_.pop();
      CHECK(current_cloud);
      for( size_t i = 0; i < imu_size; ++i ) {
        auto& imu_motion = imus_motion_data_[i];
        CHECK_EQ(size, imu_motion.size());
        current_imu_motions[i] = imu_motion.front();
        imu_motion.pop();
      }
    }
    if( !got_first_data ) {
      lidar_poses.push_back(LidarPose::Identity());
      last_cloud = current_cloud;
      for( size_t i = 0; i < imu_size; ++i ) {
        last_imu_motions[i] = current_imu_motions[i];
        imus_poses[i].push_back(current_imu_motions[i].pose);
      }
      got_first_data = true;
      SKIP_LOOP;
    }

    // step2 skip useless data
    // std::cout << current_imu_motions[0].velocity.norm() << " " 
    //   << translation_of_two_pose(current_imu_motions[0].pose,
    //         last_imu_motions[0].pose) << std::endl;
    if( current_imu_motions[0].velocity.norm() > 3.
        || translation_of_two_pose(current_imu_motions[0].pose,
            last_imu_motions[0].pose) < 0.2 ) {
      SKIP_LOOP;
    }

    // step3 
    // step3.1 get lidar pose using scan match
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    matcher->setInputTarget( last_cloud );
    matcher->setInputSource( current_cloud );
    matcher->align( guess, result );
    PRINT_DEBUG_FMT("match score: %lf", matcher->getFitnessScore());
    auto last_lidar_pose = lidar_poses.back();
    lidar_poses.push_back(last_lidar_pose*result);
    // step3.2 get imus data 
    for( size_t i = 0; i < imu_size; ++i ) {
      imus_poses[i].push_back(current_imu_motions[0].pose);
      last_imu_motions[i] = current_imu_motions[i];
    }
    last_cloud = current_cloud;
  }

  auto pose_size = lidar_poses.size();
  if( pose_size < 20 ) {
    PRINT_ERROR_FMT("too few pose: %lu", pose_size);
    imu_calib_thread_running_ = false;
    return;
  }

  auto pose_to_array = 
    []( Eigen::Matrix4f pose, double* array_6d ){
      if(!array_6d) {
        return;
      }
      Eigen::Vector6<double> vector6 = 
        common::TransformToVector6(pose).cast<double>();
      memcpy( array_6d, vector6.data(), sizeof(double)*6 ); 
    };

  auto pose_to_ceres_pose = 
    []( const Eigen::Matrix4f& pose, CeresPose& ceres_pose ) {
      ceres_pose.t = pose.block(0,3,3,1).cast<double>();
      ceres_pose.q = Eigen::Quaterniond( 
        Eigen::Matrix3d(pose.block(0,0,3,3).cast<double>()) );
    };

  ceres::LocalParameterization* quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  for( int j = 0; j < imu_size; j++ ) {
    CHECK_EQ(imus_poses[j].size(), lidar_poses.size());
    ceres::Problem problem;
#if 0
    double tf_lidar_base_link[6] = {0.};
    double tf_imu_bask_link[6] = {0.};
    pose_to_array( lidars_[0]->GetPose(), tf_lidar_base_link );
    pose_to_array( imus_[j]->GetTfPose(), tf_imu_bask_link );
    std::cout << std::setw(10) << "lidar" << " " << std::setw(10) 
      << "imu" << std::endl;
    for( int i = 0; i < 6; ++i ) {
      std::cout << std::setw(10) << tf_lidar_base_link[i] << " " 
        << std::setw(10) << tf_imu_bask_link[i] << std::endl;
    }

    auto& imu_poses = imus_poses[j];
    for( int i = 1; i < lidar_poses.size(); ++i ) {
      auto delta_lidar = lidar_poses[i];
      auto delta_imu   = imu_poses[i];
      ceres::CostFunction* cost_function =
        ModelImuToLidarCostFunctor::Create(delta_lidar, delta_imu);
      problem.AddResidualBlock( cost_function, nullptr, 
        tf_lidar_base_link, tf_imu_bask_link );
    }
#else
    CeresPose tf_lidar_base_link, tf_imu_bask_link;
    pose_to_ceres_pose( lidars_[0]->GetPose(), tf_lidar_base_link );
    pose_to_ceres_pose( imus_[j]->GetTfPose(), tf_imu_bask_link );
    std::cout << "lidar: " << tf_lidar_base_link.DebugString() << std::endl;
    std::cout << "imu: " << tf_imu_bask_link.DebugString() << std::endl;

    auto& imu_poses = imus_poses[j];

    for( int i = 1; i < lidar_poses.size(); ++i ) {
      auto delta_lidar = /* lidar_poses[i-1].inverse() * */lidar_poses[i];
      auto delta_imu   = /* imu_poses[i-1].inverse() * */imu_poses[i];
      ceres::CostFunction* cost_function =
        ModelImuToLidarCostFunctorWithQuarternion::Create(delta_lidar, 
          delta_imu);
      problem.AddResidualBlock( cost_function, nullptr, 
        tf_lidar_base_link.t.data(), tf_lidar_base_link.q.coeffs().data(),
        tf_imu_bask_link.t.data(), tf_imu_bask_link.q.coeffs().data() );
      problem.SetParameterization(tf_lidar_base_link.q.coeffs().data(),
                                 quaternion_local_parameterization);
      problem.SetParameterization(tf_imu_bask_link.q.coeffs().data(),
                                 quaternion_local_parameterization);
    }
#endif
    std::cout << "Solving the problem... " << std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "Finished sloving the problem " << std::endl;
    std::cout << summary.BriefReport() << std::endl;

#if 0
    for( int i = 0; i < 6; ++i ) {
      std::cout << std::setw(10) << tf_lidar_base_link[i] << " " 
        << std::setw(10) << tf_imu_bask_link[i] << std::endl;
    }
#else 
    std::cout << "lidar: " << tf_lidar_base_link.DebugString() << std::endl;
    std::cout << "imu: " << tf_imu_bask_link.DebugString() << std::endl;
#endif
  }
  
  PRINT_INFO("imu calibration thread exit.");
  imu_calib_thread_running_ = false;
}

void Calibrator::EndAllComputation()
{
  force_quit_ = true;
  while( scan_match_thread_running_ ) {
    usleep(100000);
  }
  scan_match_thread_->join();

  if(imu_calib_thread_) {
    while( imu_calib_thread_running_ ) {
      usleep(100000);
    }
    imu_calib_thread_->join();
  }
}

} // namespace l2l_calib