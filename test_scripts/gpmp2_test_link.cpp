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
 * \date    2020-14-09
 */
//----------------------------------------------------------------------

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