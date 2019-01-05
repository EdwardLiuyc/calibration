#include "options.h"
#include <fstream>
#include "common/macro_defines.h"
#include "common/pugixml.hpp"
#include "common/pugiconfig.hpp"
#include "common/math.h"

using namespace l2l_calib::common;

namespace l2l_calib {

int ReadOptions( const std::string& filename, 
  std::vector<DeviceCoordOption>& options )
{
  const char* config_file_name = filename.c_str();
  // step1. ensure the file is valid
  if( !config_file_name || config_file_name[0]=='\0' ) {
    PRINT_WARNING("Invalid config file name.");
    return -1;
  }
  std::ifstream file( config_file_name );
  if( !file.good() ) {
    PRINT_WARNING("File not exist.");
    return -1;
  }
  pugi::xml_document doc;
  if (!doc.load_file(config_file_name)) {
    PRINT_WARNING("Failed to load target file.");
    return -1;
  }
  pugi::xml_node nullmax_node = doc.child("nullmax");
  if (nullmax_node.empty()) {
    PRINT_WARNING("Wrong node.");
    return -1;
  }
  pugi::xml_node calib_node = nullmax_node.child("calibration");
  if (calib_node.empty() ) {
    PRINT_WARNING("Wrong node.");
    return -1;
  }

  options.clear();
  const char* device_names[kDeviceTypeCount] = {"lidar","imu"};
  for( auto device_node = calib_node.first_child();
    device_node; device_node = device_node.next_sibling()) {
    
    const char* name = device_node.name();
    DeviceType device_type;
    bool is_a_device_node = false;
    for( int i = 0; i < kDeviceTypeCount; ++i ) {
      if( std::strstr(name, device_names[i]) ) {
        is_a_device_node = true;
        device_type = (DeviceType)i;
      }
    }
    if( !is_a_device_node ) {
      PRINT_INFO("It is not a device node.");
      continue;
    }

    DeviceCoordOption option;
    option.type = device_type;
    std::cout << BOLD << "device: " << device_names[device_type] 
      << NONE_FORMAT << std::endl;
    if( device_node.attribute("topic_name") ) {
      option.topic_name = device_node.attribute("topic_name").as_string();
    }
    if( option.topic_name.empty() ) {
      PRINT_ERROR("empty topic name");
      return -1;
    }
    std::cout << "read topic name:\n  " << option.topic_name << std::endl;

    option.use_euler_angles = true;
    if(device_node.attribute("use_euler_angles")) {
      option.use_euler_angles = 
        device_node.attribute("use_euler_angles").as_bool();
    } else {
      PRINT_INFO("use angles as default.");
    }

// define for simplifying the code 
#define READ_SINGLE_CONFIG(child_name, attr_name, target) \
    target = 0.; \
    if( device_node.child(child_name).attribute(attr_name) ) { \
      target = device_node.child(child_name).attribute(attr_name).as_float(); \
    }

    // translation
    auto& trans = option.translation;
    if( device_node.child("translation") ) {
      READ_SINGLE_CONFIG("translation", "x", trans.x);
      READ_SINGLE_CONFIG("translation", "y", trans.y);
      READ_SINGLE_CONFIG("translation", "z", trans.z);
    }
    std::cout << "read translation: \n" << 
      "  x: " << trans.x << "\n" << 
      "  y: " << trans.y << "\n" << 
      "  z: " << trans.z << std::endl;

    // euler angle
    auto& q = option.quaternion;
    if( option.use_euler_angles ) {
      if( device_node.child("angles") ) {
        auto& angles = option.angles;
        READ_SINGLE_CONFIG("angles", "r", angles.r);
        READ_SINGLE_CONFIG("angles", "p", angles.p);
        READ_SINGLE_CONFIG("angles", "y", angles.y);
        std::cout << "read angles: \n" << 
          "  r: " << angles.r << "\n" << 
          "  p: " << angles.p << "\n" << 
          "  y: " << angles.y << std::endl;

        std::cout << "calculated ";
        Eigen::Vector3f angles_rad( DegToRad( angles.r ), 
          DegToRad( angles.p ), DegToRad( angles.y ));
        Eigen::Quaternionf q = EulerAnglesToQuaternion(angles_rad);
        option.quaternion.w = q.w();
        option.quaternion.x = q.x();
        option.quaternion.y = q.y();
        option.quaternion.z = q.z();
      } else {
        PRINT_ERROR("no angles");
        return -1;
      }
    } else {
      if( device_node.child("quaternion") ) {
        std::cout << "read ";
        READ_SINGLE_CONFIG("quaternion", "x", q.x);
        READ_SINGLE_CONFIG("quaternion", "y", q.y);
        READ_SINGLE_CONFIG("quaternion", "z", q.z);
        READ_SINGLE_CONFIG("quaternion", "w", q.w);
      } else {
        PRINT_ERROR("no quaternion");
        return -1;
      }
    }

    std::cout << "quaternion: \n" << 
      "  x: " << q.x << "\n" << 
      "  y: " << q.y << "\n" << 
      "  z: " << q.z << "\n" << 
      "  w: " << q.w << std::endl;

    options.push_back(option);
    std::cout << std::endl;
  }

  return 0;
}

} // namespace l2l_calib