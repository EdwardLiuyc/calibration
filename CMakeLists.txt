cmake_minimum_required(VERSION 2.8.3)
project(l2l_calib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -fPIC -Wno-sign-compare -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(src/third_parties/libpointmatcher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(glog)
find_package(Threads REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen ${CERES_EIGEN_VERSION} EXACT QUIET)

option(USE_OPENMP "Enable OpenMP?" ON) # set to OFF to disable
if(USE_OPENMP)
 FIND_PACKAGE(OpenMP)
 if(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP flags = ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 else(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP not supported")
 endif(OPENMP_FOUND OR OpenMP_FOUND)
endif(USE_OPENMP)

include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIR})
include_directories(
  src
  src/third_parties
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

file(GLOB srcs "src/*.cc")
file(GLOB common_srcs "src/common/*.cc")
add_executable(${PROJECT_NAME}_node 
  ${srcs} 
  ${common_srcs} )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  glog 
  pointmatcher
  gtsam
  ceres
)
