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



#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <sdf_mp_integration/SDFHandler.h>
#include <sdf_mp_integration/ObstacleFactor.h>
#include <sdf_mp_integration/ObstacleFactorGP.h>


template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values MarkTrajOptimize(
    const ROBOT& arm, const SDFHandler& sdf_handler,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting) {

  using namespace gtsam;

  // GP interpolation setting
  const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
  const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);

  // build graph
  NonlinearFactorGraph graph;

  for (size_t i = 0; i <= setting.total_step; i++) {
    Key pose_key = Symbol('x', i);
    Key vel_key = Symbol('v', i);

    // start and end
    if (i == 0) {
      graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting.conf_prior_model));
      graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting.vel_prior_model));

    } else if (i == setting.total_step) {
      graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf, setting.conf_prior_model));
      graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel, setting.vel_prior_model));
    }

    if (setting.flag_pos_limit) {
      // joint position limits
      graph.add(LIMIT_FACTOR_POS(pose_key, setting.pos_limit_model, setting.joint_pos_limits_down, 
          setting.joint_pos_limits_up, setting.pos_limit_thresh));
    }
    if (setting.flag_vel_limit) {
      // velocity limits
      graph.add(LIMIT_FACTOR_VEL(vel_key, setting.vel_limit_model, setting.vel_limits, 
          setting.vel_limit_thresh));
    }

    // non-interpolated cost factor
    graph.add(OBS_FACTOR(pose_key, arm, sdf_handler, setting.cost_sigma, setting.epsilon));

    if (i > 0) {
      Key last_pose_key = Symbol('x', i-1);
      Key last_vel_key = Symbol('v', i-1);

      // interpolated cost factor
      if (setting.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting.obs_check_inter; j++) {
          const double tau = inter_dt * static_cast<double>(j);
          graph.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm, sdf_handler,
              setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
        }
      }

      // GP factor
      graph.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t,
          setting.Qc_model));
    }
  }


  return gpmp2::optimize(graph, init_values, setting);
//   return result;
}


/* ************************************************************************** */
/* main function */
int main(int argc, char **argv) {

  std::cout << "Entered main" << std:: endl;
  ros::init(argc, argv, "gpmp2_example");
  ros::NodeHandle node("~");
  ros::NodeHandle nh_private;
  std::string camera_topic = "/camera_pose";
  std::string pointcloud_topic = "/hsrb/head_rgbd_sensor/depth_registered/points";
  
  // Set up FIESTA  
  // FiestaPtr esdf_ptr; 
  // esdf_ptr = new FiestaClass(node);
  // sdf_mp_integration::SDFHandler<FiestaPtr> sdf_handler(esdf_ptr);
  // ros::spinOnce();

  // Set up Voxblox
  // VoxbloxClass voxblox_node(node, nh_private);
  // VoxbloxPtr voxblox_esdf_ptr = voxblox_node.getEsdfMapPtr();
  // sdf_mp_integration::SDFHandler<VoxbloxPtr> sdf_handler(voxblox_esdf_ptr);
  // ros::spinOnce();
 
  // Set up GPU-Voxels
  GPUVoxelsPtr gpu_voxels_ptr; 
  gpu_voxels_ptr = new gpu_voxels_ros::GPUVoxelsServer(node);
  sdf_mp_integration::SDFHandler<GPUVoxelsPtr> sdf_handler(gpu_voxels_ptr);
  ros::spinOnce();
  ros::Rate r(30);

  std::cout << "Successfully set up the mapping server." << std:: endl;

  // 2 link simple example, with none zero base poses
  gtsam::Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  // gtsam::Pose3 base_pose(Rot3(), Point3(2.0, 1.0, -1.0));
  gtsam::Pose3 base_pose(Rot3(), Point3(2.0, 1.0, 1.0));
  gpmp2::Arm abs_arm(2, a, alpha, d, base_pose);
  
  // body spheres
  gpmp2::BodySphereVector body_spheres;
  body_spheres.push_back(gpmp2::BodySphere(0, 0.5, gtsam::Point3(-1.0, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(-0.5, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(0, 0.1, gtsam::Point3(0, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(-0.5, 0, 0)));
  body_spheres.push_back(gpmp2::BodySphere(1, 0.1, gtsam::Point3(0, 0, 0)));
  gpmp2::ArmModel arm(abs_arm, body_spheres);
  
  std::cout << "Created arm model." << std:: endl;

  // settings
  double pose_fix_sigma = 0.0001;
  double vel_fix_sigma = 0.0001; 
  size_t total_step = 10;
  gtsam::Matrix2 Qc = 1 * gtsam::Matrix2::Identity(2,2);

  gpmp2::TrajOptimizerSetting setting(2);
  setting.setGaussNewton();
  setting.set_total_step(total_step);
  setting.set_total_time(10);
  setting.set_epsilon(0.2);
  setting.set_cost_sigma(0.2);
  setting.set_obs_check_inter(1);
  setting.set_conf_prior_model(pose_fix_sigma);
  setting.set_vel_prior_model(vel_fix_sigma);
  setting.set_Qc_model(Qc);


  gtsam::Vector2 start_conf(0.0, 0.0);
  gtsam::Vector2 end_conf(0.0, 1.57);
  gtsam::Vector start_vel = gtsam::Vector::Zero(2);
  gtsam::Vector end_vel = gtsam::Vector::Zero(2);

  gtsam::Values init_values = gpmp2::initArmTrajStraightLine(start_conf, end_conf, total_step);


  std::cout << "Starting optimisation." << std:: endl;

  // gtsam::Values res = MarkTrajOptimize<gpmp2::ArmModel, gpmp2::GaussianProcessPriorLinear, sdf_mp_integration::SDFHandler<FiestaPtr>, 
  //                                     sdf_mp_integration::ObstacleFactor<FiestaPtr, gpmp2::ArmModel>, 
  //                                     sdf_mp_integration::ObstacleFactorGP<FiestaPtr, gpmp2::ArmModel, gpmp2::GaussianProcessInterpolatorLinear> , 
  //                                     gpmp2::JointLimitFactorVector, gpmp2::VelocityLimitFactorVector>(arm, sdf_handler, start_conf, start_vel, end_conf, end_vel, init_values, setting);
  
  // gtsam::Values res = MarkTrajOptimize<gpmp2::ArmModel, gpmp2::GaussianProcessPriorLinear, sdf_mp_integration::SDFHandler<VoxbloxPtr>, 
  //                                     sdf_mp_integration::ObstacleFactor<VoxbloxPtr, gpmp2::ArmModel>, 
  //                                     sdf_mp_integration::ObstacleFactorGP<VoxbloxPtr, gpmp2::ArmModel, gpmp2::GaussianProcessInterpolatorLinear> , 
  //                                     gpmp2::JointLimitFactorVector, gpmp2::VelocityLimitFactorVector>(arm, sdf_handler, start_conf, start_vel, end_conf, end_vel, init_values, setting);

  // gtsam::Values res = MarkTrajOptimize<gpmp2::ArmModel, gpmp2::GaussianProcessPriorLinear, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
  //                                     sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::ArmModel>, 
  //                                     sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::ArmModel, gpmp2::GaussianProcessInterpolatorLinear> , 
  //                                     gpmp2::JointLimitFactorVector, gpmp2::VelocityLimitFactorVector>(arm, sdf_handler, start_conf, start_vel, end_conf, end_vel, init_values, setting);

  // res.print();

  std::cout << "Finished!" << std::endl;
  
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}


