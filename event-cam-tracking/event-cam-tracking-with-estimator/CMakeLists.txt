project(metavision_sdk_get_started)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_FIND_DEBUG_MODE 1)

include_directories(${./})

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(OpenCL)

find_package(Eigen3 QUIET)
if (NOT Eigen3_FOUND)
  set(EIGEN_INCLUDE_DIR_HINTS ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)
  set(OpenMVG_USE_INTERNAL_EIGEN ON)
  find_package(Eigen QUIET)
  message(STATUS "Eigen3 not found, using internal version")
else()
  set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  message(STATUS "Eigen3 found")
endif()

INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})

# set (sample metavision_sdk_get_started2_events_clustering)
# set (sample metavision_sdk_get_started5_opencl_store)
set (sample event_data_cluster_motion_estimation_vel_lost)
add_executable(${sample} ${sample}.cpp AEClustering.cpp MyCluster.cpp)
target_link_libraries(${sample} MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui OpenCL::OpenCL)
