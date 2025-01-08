project(metavision_sdk_get_started)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

include_directories(${./})

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
find_package(OpenCL)

# set (sample metavision_sdk_get_started2_events_clustering)
set (sample metavision_sdk_get_started5_opencl_store)
add_executable(${sample} ${sample}.cpp AEClustering.cpp MyCluster.cpp)
target_link_libraries(${sample} MetavisionSDK::core MetavisionSDK::driver MetavisionSDK::ui OpenCL::OpenCL)
