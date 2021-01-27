#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <sdf_mp_integration/ArmPose.h>
#include <sdf_mp_integration/WholeBodyPose.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "arm_goal_test_node");

  ros::NodeHandle node;
  ros::Rate loop_rate(1);

  ros::Publisher pub = node.advertise<sdf_mp_integration::ArmPose>("arm_goal", 10);
	
  std_msgs::Float32MultiArray array;

  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // array.layout.dim[0].label = "";
  // array.layout.dim[0].size = 0;
  // array.layout.dim[0].stride = 0;
  // array.layout.data_offset = 0;

  array.data.resize(5);
  array.data[0] = 0.0;
  array.data[1] = 0.0;
  array.data[2] = 0.0;
  array.data[3] = 0.0;
  array.data[4] = 0.0;

  while (ros::ok())
  {
    pub.publish(array);    
    ROS_INFO("Published the waypoints");

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
};