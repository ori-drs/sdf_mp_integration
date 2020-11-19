/*
 * dummy_motion_planner.cpp
 *
 *  Created on: 08/09/2020
 *  Author: Mark Finean
 */
#include "sdf_mp_integration/dummy_motion_planner.h"

namespace dummy
{
DummyMotionPlanner::DummyMotionPlanner(fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* esdf_ptr)
{
  ROS_INFO("DummyMotionPlanner Constructor");
  esdf_ptr_ = esdf_ptr;
}

DummyMotionPlanner::~DummyMotionPlanner()
{
  ROS_INFO("DummyMotionPlanner Destructor");
}

double DummyMotionPlanner::GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad)
{
      
  // auto start = std::chrono::high_resolution_clock::now(); 

  double dist;

  dist = esdf_ptr_->GetDistanceAndGradient(pos, grad);

  // auto finish = std::chrono::high_resolution_clock::now(); 

  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start); 
  // double dur = duration.count();
  // ROS_INFO("Actual query time: %f microseconds.", dur);

  ROS_INFO("DummyMotionPlanner finished querying");
  return dist;
}

        // double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);


} // namespace dummy
