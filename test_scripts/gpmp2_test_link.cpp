// #include "Fiesta.h"
// #include <gpmp2/utils/angles.h>


#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
// #include <gpmp2/planner/ISAM2TrajOptimizer.h>
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/RobotModel.h>
#include <gpmp2/kinematics/ArmModel.h>

#include <iostream>

int main(int argc, char **argv) {


    // double rads = angles::from_degrees(180);
    // std::cout << rads << std::endl;
    std::cout << "Started" << std::endl;

    gtsam::Vector2 a(1, 1), alpha(0, 0), d(0, 0);
    std::cout << "Done vectors" << std::endl;
   // gtsam::Pose3 base_pose(gtsam::Rot3(), gtsam::Point3(2.0, 1.0, -1.0));
    // gpmp2::Arm abs_arm();
    gpmp2::Arm abs_arm(2, a, alpha, d);
    std::cout << "Arm initialised" << std::endl;


    // body spheres
    gpmp2::BodySphereVector body_spheres;
    body_spheres.push_back(gpmp2::BodySphere(0, 0.5, gtsam::Point3(-1.0, 0, 0)));
    body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(-0.5, 0, 0)));
    body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(0, 0, 0)));
    body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(-0.5, 0, 0)));
    body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(0, 0, 0)));
    gpmp2::ArmModel arm(abs_arm, body_spheres);
    std::cout << "Arm model created" << std::endl;

    abs_arm.~Arm();
    std::cout << "Arm destroyed" << std::endl;
    std::cout << "Finished" << std::endl;

  return 0;
}