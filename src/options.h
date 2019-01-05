#ifndef L2L_CALIB_OPTIONS_H_
#define L2L_CALIB_OPTIONS_H_

#include <string>
#include <vector>

namespace l2l_calib {

enum DeviceType {
  kLidar, kImu, kDeviceTypeCount
};

struct DeviceCoordOption {
  std::string topic_name;
  bool use_euler_angles;
  DeviceType type;

  struct {
    float x,y,z;
  }translation;

  struct {
    float r,p,y;
  }angles;

  struct {
    float x,y,z,w;
  }quaternion;
};

int ReadOptions( const std::string& filename, 
  std::vector<DeviceCoordOption>& options );

} // namespace l2l_calib

#endif