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