/*
 * fiesta_nodelet.cpp
 *
 *  Created on: 08/09/2020
 *  Author: Mark Finean
 */
#include "sample_nodelet/fiesta_nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace fiesta
{
FiestaNodelet::FiestaNodelet()
{
  ROS_INFO("FiestaNodelet Constructor");
}

FiestaNodelet::~FiestaNodelet()
{
  ROS_INFO("FiestaNodelet Destructor");
}

void FiestaNodelet::onInit()
{
  // fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> esdf_map(node);
  NODELET_INFO("FiestaNodelet - %s", __FUNCTION__);

  inst_.reset(new fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>(getPrivateNodeHandle()));  

}
} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(fiesta::FiestaNodelet, nodelet::Nodelet)
