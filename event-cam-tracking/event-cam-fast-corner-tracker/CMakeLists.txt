# Copyright (c) Prophesee S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

find_package(MetavisionSDK COMPONENTS core driver ui REQUIRED)
set (sample metavision_time_surface_periodic_group_track )
set (common_libraries MetavisionSDK::driver MetavisionSDK::core MetavisionSDK::ui Boost::program_options)

find_package(OpenCL)
find_package( OpenCV REQUIRED )
if(NOT OpenCV_FOUND)
  message(STATUS "cannot find OpenCV.cmake fallback to pkg-config")
endif(NOT OpenCV_FOUND)

find_package(Boost COMPONENTS program_options)


add_executable(${sample} ${sample}.cpp)
target_link_libraries(${sample} PRIVATE ${common_libraries} ${OpenCV_LIBS} OpenCL::OpenCL Boost::program_options)

install(FILES ${sample}.cpp
        DESTINATION share/metavision/sdk/core/cpp_samples/${sample}
        COMPONENT metavision-sdk-core-samples
)

install(FILES CMakeLists.txt.install
        RENAME CMakeLists.txt
        DESTINATION share/metavision/sdk/core/cpp_samples/${sample}
        COMPONENT metavision-sdk-core-samples
)
