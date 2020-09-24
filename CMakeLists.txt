cmake_minimum_required(VERSION 3.0.2)
project(sample_nodelet)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall ") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "DEBUG")

find_package(catkin COMPONENTS voxblox voxblox_ros voxblox_msgs voxblox_rviz_plugin REQUIRED)
find_package(roscpp REQUIRED)
find_package(nodelet REQUIRED)
find_package(fiesta REQUIRED)
find_package(gpmp2 REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED) # Why this is needed, i have no idea

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${nodelet_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${fiesta_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gpmp2_INCLUDE_DIR}
  # ${voxblox_ros_INCLUDE_DIRs}
)

add_library(${PROJECT_NAME}_lib
  src/dummy_motion_planner.cpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nodelet fiesta voxblox_ros voxblox voxblox_msgs voxblox_rviz_plugin 
)





# add_executable(fiesta_tester src/fiesta_tester.cpp)
# target_link_libraries(fiesta_tester 
# ${catkin_LIBRARIES} 
# ${roscpp_LIBRARIES} 
# ${nodelet_LIBRARIES}
# ${fiesta_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBS}
# ${PROJECT_NAME}_lib
# )



# add_executable(gpmp2_test_link src/gpmp2_test_link.cpp)
# target_link_libraries(gpmp2_test_link 
# ${catkin_LIBRARIES} 
# ${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBS}
# gpmp2
# )


# add_executable(fiesta_gpmp2_node  src/fiesta_gpmp2.cpp src/sdf_handler.cpp)

# target_link_libraries(fiesta_gpmp2_node 
# ${catkin_LIBRARIES} 
# ${roscpp_LIBRARIES} 
# ${fiesta_LIBRARIES}
# ${PCL_LIBRARIES}
# ${OpenCV_LIBS}
# gpmp2
# gtsam
# )

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
)


# add_executable(class_test src/class_test_node.cpp src/class_test.cpp)
# target_link_libraries(class_test 
# ${catkin_LIBRARIES} 
# ${roscpp_LIBRARIES} 
# )




add_library(sample_nodelet
  src/fiesta_nodelet.cpp
  src/sample_nodelet_class.cpp
  src/sample_nodelet_class2.cpp
)

target_link_libraries(sample_nodelet ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${nodelet_LIBRARIES} ${fiesta_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBS})




if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(sample_nodelet ${catkin_EXPORTED_LIBRARIES}  ${roscpp_LIBRARIES} ${nodelet_LIBRARIES} ${fiesta_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS})
endif()


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
