#ifndef L2L_CALIB_CALIBRATOR_H_
#define L2L_CALIB_CALIBRATOR_H_

#include "lidar.h"
#include "imu.h"
#include "registrators/registrator_interface.h"
#include <queue>

namespace l2l_calib {

struct TfSetting {
  std::string topic_name;
  Eigen::Vector3f translation;
  Eigen::Quaternionf quaternion;
};
typedef TfSetting LidarSetting;
typedef TfSetting ImuSetting;

struct CalibratorSettings {
  std::vector<LidarSetting> lidar_settings;
  std::vector<ImuSetting> imu_settings;
};

class Calibrator {
public:
  Calibrator() = default;
  ~Calibrator() = default;

  using ShowFusedCloudFunc = 
    std::function<void(const Lidar::PointCloudPtr&)>;

  void Initialise( ros::NodeHandle& nh, const CalibratorSettings& settings );
  void EndAllComputation();

  inline 
  void SetShowFunction( ShowFusedCloudFunc func ) 
  { 
    show_fused_cloud_function_ = func; 
  }

protected:
  void ScanMatching();
  void ImuCalibrating();

private:
  // lidars
  std::vector<std::shared_ptr<Lidar>> lidars_;
  std::shared_ptr<std::thread> scan_match_thread_;
  std::shared_ptr<registrator::RegistratorInterface<Lidar::PointType>>
    scan_matcher_;

  ShowFusedCloudFunc show_fused_cloud_function_;

  // imus 
  struct ImuMotionData {
    Imu::Pose pose;
    Imu::Velocity3 velocity;
  };
  common::Mutex mutex_;
  std::vector<std::shared_ptr<Imu>> imus_;
  std::queue<Lidar::PointCloudPtr> clouds_for_imu_calib_;
  std::vector<std::queue<ImuMotionData>> imus_motion_data_;
  std::shared_ptr<std::thread> imu_calib_thread_;

  bool lidar_calibration_finished_ = false;
  bool imu_calibration_finished_ = false;
  bool scan_match_thread_running_ = false;
  bool imu_calib_thread_running_ = false;
  bool force_quit_ = false;
};

}

#endif