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

#include "Fiesta.h"
#include <sdf_mp_integration/dummy_motion_planner.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "FIESTA");
  ros::NodeHandle node("~");
  fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* esdf_ptr; 
  esdf_ptr = new fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>(node);
  
  ros::spinOnce(); 

  dummy::DummyMotionPlanner motion_planner(esdf_ptr);

  double dist;
  double dur;

  for (size_t i = 0; i < 100; i++)
  {
    Eigen::Vector3d grad;  
    Eigen::Vector3d pos(1+i*0.01,0,0);

    auto start = std::chrono::high_resolution_clock::now(); 

    dist = motion_planner.GetDistanceAndGradient(pos, grad);

    auto finish = std::chrono::high_resolution_clock::now(); 

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start); 
    dur = duration.count();


    ROS_INFO("Actual query time: %f microseconds.", dur);

  }
  

  ros::spin();
  return 0;
}