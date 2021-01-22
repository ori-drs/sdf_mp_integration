#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "transform_republish_node");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::TransformStamped>("transform_topic", 10);

  tf::TransformListener listener;

  ros::Rate rate(30.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/head_rgbd_sensor_rgb_frame",  
                               ros::Time(0), transform);

      geometry_msgs::TransformStamped msg;
      tf::transformStampedTFToMsg(transform, msg);
      pub.publish(msg);

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    rate.sleep();
  }
  return 0;
};