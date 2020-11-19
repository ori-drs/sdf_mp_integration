cmake_minimum_required(VERSION 3.0.2)
project(sdf_mp_integration)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall ") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin COMPONENTS voxblox voxblox_ros voxblox_msgs voxblox_rviz_plugin REQUIRED)
find_package(roscpp REQUIRED)
find_package(fiesta REQUIRED)
find_package(gpu_voxels_ros REQUIRED)
find_package(gpmp2 REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED) # Why this is needed, i have no idea
find_package(gpu_voxels PATHS /home/mark/installs/gpu-voxels/export REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(icl_core PATHS /home/mark/installs/gpu-voxels/export REQUIRED )
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(rospy REQUIRED)

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)


add_definitions(
  ${gpu_voxels_DEFINITIONS}
  ${icl_core_DEFINITIONS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${fiesta_INCLUDE_DIRS}
  ${gpu_voxels_ros_INCLUDE_DIRS}
  ${gpu_voxels_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gpmp2_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/dummy_motion_planner.cpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp fiesta  voxblox_ros voxblox voxblox_msgs voxblox_rviz_plugin 
)

add_executable(gpmp2_example  src/gpmp2_example.cpp src/sdf_handler.cpp)

target_link_libraries(gpmp2_example 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
${fiesta_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS}
gpmp2
gtsam
${voxblox_ros_LIBRARIES}

${Boost_SYSTEM_LIBRARY}
${icl_core_LIBRARIES}
${gpu_voxels_LIBRARIES}
${CUDA_LIBRARIES}
${orocos_kdl_LIBRARIES}
${rospy_LIBRARIES}
${gpu_voxels_ros_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
