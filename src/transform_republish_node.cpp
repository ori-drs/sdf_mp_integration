// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the sdf_mp_integration package.
// © Copyright 2022, Mark Finean 
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the followingdisclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation 
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software 
// without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Finean
 * \date    2021-12-01
 */
//----------------------------------------------------------------------

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