#include <gtsam/base/Matrix.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <iostream>
#include <ros/ros.h>
#include <sdf_mp_integration/PlanningServer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pose2vector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    std::cout << "Starting test." << std::endl;
    gpmp2::Pose2Vector p1;     // default nothing
    gpmp2::Pose2Vector pi(gtsam::Pose2(), gtsam::Vector::Zero(5));     // manual identity
    p1 = pi;  // assignment

    std::cout << "Finished test." << std::endl;

    return 0;
}