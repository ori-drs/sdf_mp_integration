cmake_minimum_required(VERSION 3.0.2)
project(sdf_mp_integration)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
sensor_msgs
control_msgs
nav_msgs
tf
tmc_omni_path_follower
message_generation # For message generation
)

# find_package(catkin COMPONENTS voxblox voxblox_ros voxblox_msgs voxblox_rviz_plugin REQUIRED)
find_package(roscpp REQUIRED)
# find_package(fiesta REQUIRED)
find_package(gpu_voxels_ros REQUIRED)
find_package(gpmp2 REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED) # Why this is needed, i have no idea
find_package(gpu_voxels PATHS $ENV{GPU_VOXELS_INSTALL_DIR} REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(icl_core PATHS $ENV{GPU_VOXELS_INSTALL_DIR} REQUIRED )
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

find_package(ompl REQUIRED)

add_definitions(
  ${gpu_voxels_DEFINITIONS}
  ${icl_core_DEFINITIONS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  # ${fiesta_INCLUDE_DIRS}
  ${gpu_voxels_ros_INCLUDE_DIRS}
  ${gpu_voxels_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gpmp2_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${rospy_INCLUDE_DIRS}
  ${dgpmp2_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  # src/dummy_motion_planner.cpp
  src/SDFHandler.cpp
  src/PlanningServer.cpp
  src/GraphMaintainer.cpp
  src/SetHSRConf.cpp
  src/GenerateArm.cpp
  src/HeadController.cpp
  src/ResultsRecorder.cpp
  src/utils/timing.cpp
  src/utils/traj_utils.cpp
  src/utils/Visualiser.cpp
)

add_message_files(
  FILES
  GtsamValue.msg
  GtsamValues.msg
  WholeBodyPose.msg
  ArmPose.msg
  HeadDirection.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp   
  message_runtime # for message generation
  # fiesta 
  # voxblox_ros voxblox voxblox_msgs voxblox_rviz_plugin 
)


# add_executable(gpmp2_example test_scripts/gpmp2_example.cpp 
# src/SDFHandler.cpp)

# target_link_libraries(gpmp2_example 
# ${catkin_LIBRARIES} 
# ${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBS}
# gpmp2
# gtsam
# ${voxblox_ros_LIBRARIES}

# ${Boost_SYSTEM_LIBRARY}
# ${icl_core_LIBRARIES}
# ${gpu_voxels_LIBRARIES}
# ${CUDA_LIBRARIES}
# ${orocos_kdl_LIBRARIES}
# ${rospy_LIBRARIES}
# ${gpu_voxels_ros_LIBRARIES}
# )

# Add full server node for the HSR

add_executable(planning_server_node test_scripts/planning_server_node.cpp 
src/SDFHandler.cpp
src/PlanningServer.cpp
src/GraphMaintainer.cpp
src/SetHSRConf.cpp
src/GenerateArm.cpp
src/ResultsRecorder.cpp
src/HeadController.cpp
src/utils/timing.cpp
src/utils/traj_utils.cpp
src/utils/Visualiser.cpp
)

target_link_libraries(planning_server_node 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS}
gpmp2
gtsam
# ${voxblox_ros_LIBRARIES}

${Boost_SYSTEM_LIBRARY}
${icl_core_LIBRARIES}
${gpu_voxels_LIBRARIES}
${CUDA_LIBRARIES}
${orocos_kdl_LIBRARIES}
${rospy_LIBRARIES}
${gpu_voxels_ros_LIBRARIES}
${dgpmp2_LIBRARIES}
${OMPL_LIBRARIES}

# ${PROJECT_NAME}_lib
)

add_executable(composite_planning_server_node test_scripts/composite_planning_server_node.cpp 
src/SDFHandler.cpp
src/PlanningServer.cpp
src/GraphMaintainer.cpp
src/SetHSRConf.cpp
src/GenerateArm.cpp
src/ResultsRecorder.cpp
src/HeadController.cpp
src/utils/timing.cpp
src/utils/traj_utils.cpp
src/utils/Visualiser.cpp
)

target_link_libraries(composite_planning_server_node 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS}
gpmp2
gtsam
# ${voxblox_ros_LIBRARIES}

${Boost_SYSTEM_LIBRARY}
${icl_core_LIBRARIES}
${gpu_voxels_LIBRARIES}
${CUDA_LIBRARIES}
${orocos_kdl_LIBRARIES}
${rospy_LIBRARIES}
${gpu_voxels_ros_LIBRARIES}
${dgpmp2_LIBRARIES}
${OMPL_LIBRARIES}

# ${PROJECT_NAME}_lib
)

add_executable(single_composite_planning_server_node test_scripts/single_composite_planning_server_node.cpp 
src/SDFHandler.cpp
src/PlanningServer.cpp
src/GraphMaintainer.cpp
src/SetHSRConf.cpp
src/GenerateArm.cpp
src/ResultsRecorder.cpp
src/HeadController.cpp
src/utils/timing.cpp
src/utils/traj_utils.cpp
src/utils/Visualiser.cpp
)

target_link_libraries(single_composite_planning_server_node 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS}
gpmp2
gtsam
# ${voxblox_ros_LIBRARIES}

${Boost_SYSTEM_LIBRARY}
${icl_core_LIBRARIES}
${gpu_voxels_LIBRARIES}
${CUDA_LIBRARIES}
${orocos_kdl_LIBRARIES}
${rospy_LIBRARIES}
${gpu_voxels_ros_LIBRARIES}
${dgpmp2_LIBRARIES}
${OMPL_LIBRARIES}

# ${PROJECT_NAME}_lib
)

add_executable(test_pose2vector test_scripts/test_pose2vector.cpp 
)
target_link_libraries(test_pose2vector 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
gpmp2
gtsam
${CUDA_LIBRARIES}

)


add_executable(transform_republish_node src/transform_republish_node.cpp)
target_link_libraries(transform_republish_node 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
)

add_executable(arm_goal_test_node scripts/arm_goal_test_node.cpp)
target_link_libraries(arm_goal_test_node 
${catkin_LIBRARIES} 
${roscpp_LIBRARIES} 
)

add_dependencies(arm_goal_test_node sdf_mp_integration_generate_messages_cpp)

catkin_install_python(PROGRAMS  scripts/plotting_node.py 
                                scripts/hsr_python_controller.py
                                scripts/moving_obstacle_node.py
                                scripts/nbv_moving_obstacle_node.py
                                scripts/person_moving_obstacle_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Install directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
PATTERN ".svn" EXCLUDE)

install(DIRECTORY worlds/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
PATTERN ".svn" EXCLUDE)

install(DIRECTORY maps/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
PATTERN ".svn" EXCLUDE)