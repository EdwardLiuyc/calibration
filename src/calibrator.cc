#include "calibrator.h"
#include "sensor_msgs/PointCloud2.h"
#include "registrators/icp_pointmatcher.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "cost_function.h"

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
  CHECK_GE( lidar_settings.size(), 2 );
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
    imus_.resize(imu_settings.size());
    imus_motion_data_.resize(imu_settings.size());
    for( size_t i = 0; i < imu_settings.size(); ++i ) {
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
  std::queue<LidarPose> lidar_poses;
  std::vector<std::queue<Imu::Pose>> imus_poses;
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
      clouds_for_imu_calib_.pop();
      CHECK(current_cloud);
      auto size = clouds_for_imu_calib_.size();
      for( size_t i = 0; i < imu_size; ++i ) {
        auto& imu_motion = imus_motion_data_[i];
        CHECK_EQ(size, imu_motion.size());
        current_imu_motions[i] = imu_motion.front();
        imu_motion.pop();
      }
    }
    if( !got_first_data ) {
      lidar_poses.push(LidarPose::Identity());
      last_cloud = current_cloud;
      for( size_t i = 0; i < imu_size; ++i ) {
        last_imu_motions[i] = current_imu_motions[i];
        imus_poses[i].push(current_imu_motions[i].pose);
      }
      got_first_data = true;
      SKIP_LOOP;
    }

    // step2 skip useless data
    if( current_imu_motions[0].velocity.norm() > 0.5 
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
    lidar_poses.push(last_lidar_pose*result);
    // step3.2 get imus data 
    for( size_t i = 0; i < imu_size; ++i ) {
      imus_poses[i].push(current_imu_motions[0].pose);
      CHECK_EQ(imus_poses[i].size(), lidar_poses.size());
    }
  }

  auto pose_size = lidar_poses.size();
  if( pose_size < 20 ) {
    PRINT_ERROR("too few pose");
    imu_calib_thread_running_ = false;
    return;
  }

  std::vector<ceres::Problem> problems;
  problems.resize(imu_size);
  while( !lidar_poses.empty() ){
    auto lidar_pose = lidar_poses.front();
    lidar_poses.pop();

    for( size_t i = 0; i < imu_size; ++i ) {
      auto& problem = problems[i];
      auto& imu_poses = imus_poses[i];
      auto imu_pose = imu_poses.front();
      imu_poses.pop();
      ceres::CostFunction* cost_function =
        ModelImuToLidarCostFunctor::Create(lidar_pose, imu_pose);

      problem.AddResidualBlock(cost_function, NULL,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());
    }

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