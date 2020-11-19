#include <sdf_mp_integration/sdf_handler.h>
#include <sdf_mp_integration/ObstacleFactor.h>

#include <iostream>
// #include <chrono>

#include <gtsam/base/Vector.h>
// #include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
#include "Fiesta.h"

#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gpmp2/kinematics/ArmModel.h>

typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* FiestaPtr;
typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> FiestaClass;

int main(int argc, char **argv) {

  std::cout << "Entered main" << std:: endl;
  ros::init(argc, argv, "FIESTA");

  // Run SDFs on spinner1
  ros::NodeHandle node("~");


  // Set up FIESTA  
  FiestaPtr esdf_ptr; 
  esdf_ptr = new FiestaClass(node);

  // Pass FIESTA to a handle for GPMP2
  sdf_mp_integration::SDFHandler<FiestaPtr> sdf_handler(esdf_ptr);
  ros::spinOnce();

  double dist;
  double dur;


  gtsam::Point3 pos(1,0,0);
  gtsam::Vector3 grad;

  sdf_handler.print();
  dist = sdf_handler.getSignedDistance(pos);

  auto start = std::chrono::high_resolution_clock::now(); 
  dist = sdf_handler.getSignedDistance(pos, grad);

  auto finish = std::chrono::high_resolution_clock::now(); 

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start); 
  dur = duration.count();


  ROS_INFO("Actual query time: %f microseconds.", dur);







  gtsam::Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  std::cout << "Done vectors" << std::endl;

  gtsam::Rot3 rot = gtsam::Rot3(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
  gtsam::Pose3 base_pose(gtsam::Rot3(), gtsam::Point3(2.0, 1.0, -1.0));


  // body spheres
  gpmp2::BodySphereVector body_spheres;
  body_spheres.push_back(gpmp2::BodySphere(0, 0.5, gtsam::Point3(-1.0, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(-0.5, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(0, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(-0.5, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(0, 0, 0)));


  gpmp2::Arm abs_arm(2, a, alpha, d, base_pose);
  std::cout << "Arm initialised" << std::endl;
  gpmp2::ArmModel arm(abs_arm, body_spheres);
  std::cout << "Arm model created" << std::endl;

  abs_arm.~Arm();
  std::cout << "Arm destroyed" << std::endl;
  std::cout << "Finished" << std::endl;
  double obs_eps = 0.2;

  sdf_mp_integration::ObstacleFactor<FiestaPtr, gpmp2::ArmModel> factor(0, arm, sdf_handler, 1.0, obs_eps);
  std::cout << "Finished" << std::endl;

  // ros::spin();


  return 0;
}