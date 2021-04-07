#include <iostream>
#include <string>
#include <chrono>

#include <dgpmp2/Visualiser.h>

#include <dgpmp2/GenerateArm.h>
#include <dgpmp2/SetHSRConf.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "dh_visualiser");

    ros::NodeHandle nh("~");

    ros::Rate r(1);

    int robot_id(1);

    gpmp2::Pose2MobileVetLinArmModel arm_model = GenerateHSRArm();

    HSRVisualiser vis(nh);
    vis.setArm(arm_model);

    gtsam::Vector conf = dgpmp2::SetHSRConf("neutral");
    gpmp2::Pose2Vector hsr_pose(gtsam::Pose2(0,0,0), conf);

    while (ros::ok())
    {
        vis.visualiseRobot(hsr_pose, robot_id);
        // vis.visualiseRobotAxes(hsr_pose, robot_id);
        r.sleep();
    }
