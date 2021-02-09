#include <sdf_mp_integration/GraphMaintainer.h>

// #include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>

// #include <gpmp2/kinematics/ArmModel.h>
// #include <gpmp2/gp/GaussianProcessPriorLinear.h>
// #include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

#include <sdf_mp_integration/SDFHandler.h>
#include <sdf_mp_integration/ObstacleFactor.h>
#include <sdf_mp_integration/ObstacleFactorGP.h>


template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
sdf_mp_integration::GraphMaintainer<ROBOT, GP, SDFHandler, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::GraphMaintainer(
  const ROBOT& arm, const SDFHandler& sdf_handler,
  const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
  const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
  const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting){
    
    using namespace gtsam;

    setting_ = setting;
    arm_ = arm; 
    start_conf_ = start_conf;
    start_vel_ = start_vel;
    end_conf_ = end_conf;
    end_vel_ = end_vel;
    init_values_ = init_values;

    delta_t_ = setting_.total_time / static_cast<double>(setting_.total_step);
    inter_dt_ = delta_t_ / static_cast<double>(setting_.obs_check_inter + 1);

    // constructGraph();


    // build graph  
    for (size_t i = 0; i < setting_.total_step; i++) {
      Key pose_key = Symbol('x', i);
      Key vel_key = Symbol('v', i);

      // start and end
      if (i == 0) {

        graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf_, setting_.conf_prior_model));
        graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel_, setting_.vel_prior_model));

      } else if (i == setting_.total_step - 1) {
        graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf_, setting_.conf_prior_model));
        graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel_, setting_.vel_prior_model));
      }

      if (setting_.flag_pos_limit) {
        // joint position limits
        graph_.add(LIMIT_FACTOR_POS(pose_key, setting_.pos_limit_model, setting_.joint_pos_limits_down, 
            setting_.joint_pos_limits_up, setting_.pos_limit_thresh));
      }
      if (setting_.flag_vel_limit) {
        // velocity limits
        graph_.add(LIMIT_FACTOR_VEL(vel_key, setting_.vel_limit_model, setting_.vel_limits, 
            setting_.vel_limit_thresh));
      }

      // non-interpolated cost factor
      graph_.add(OBS_FACTOR(pose_key, arm_, sdf_handler, setting_.cost_sigma, setting_.epsilon));

      if (i > 0) {
        Key last_pose_key = Symbol('x', i-1);
        Key last_vel_key = Symbol('v', i-1);

        // interpolated cost factor
        if (setting_.obs_check_inter > 0) {
          for (size_t j = 1; j <= setting_.obs_check_inter; j++) {
            const double tau = inter_dt_ * static_cast<double>(j);
            graph_.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm_, sdf_handler,
                setting_.cost_sigma, setting_.epsilon, setting_.Qc_model, delta_t_, tau));
          }
        }

        // GP factor
        graph_.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t_, setting_.Qc_model));
      }
    }

    graph_.print();

    // graph_.printErrors();
}

// template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
// void sdf_mp_integration::GraphMaintainer<ROBOT, GP, SDFHandler, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::constructGraph(){

//   using namespace gtsam;

//   // build graph  
//   for (size_t i = 0; i < setting_.total_step; i++) {
//     Key pose_key = Symbol('x', i);
//     Key vel_key = Symbol('v', i);

//     // start and end
//     if (i == 0) {

//       graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf_, setting_.conf_prior_model));
//       graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel_, setting_.vel_prior_model));

//     } else if (i == setting_.total_step - 1) {
//       graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf_, setting_.conf_prior_model));
//       graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel_, setting_.vel_prior_model));
//     }

//     if (setting_.flag_pos_limit) {
//       // joint position limits
//       graph_.add(LIMIT_FACTOR_POS(pose_key, setting_.pos_limit_model, setting_.joint_pos_limits_down, 
//           setting_.joint_pos_limits_up, setting_.pos_limit_thresh));
//     }
//     if (setting_.flag_vel_limit) {
//       // velocity limits
//       graph_.add(LIMIT_FACTOR_VEL(vel_key, setting_.vel_limit_model, setting_.vel_limits, 
//           setting_.vel_limit_thresh));
//     }

//     // non-interpolated cost factor
//     graph_.add(OBS_FACTOR(pose_key, arm_, sdf_handler, setting_.cost_sigma, setting_.epsilon));

//     if (i > 0) {
//       Key last_pose_key = Symbol('x', i-1);
//       Key last_vel_key = Symbol('v', i-1);

//       // interpolated cost factor
//       if (setting_.obs_check_inter > 0) {
//         for (size_t j = 1; j <= setting_.obs_check_inter; j++) {
//           const double tau = inter_dt_ * static_cast<double>(j);
//           graph_.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm_, sdf_handler,
//               setting_.cost_sigma, setting_.epsilon, setting_.Qc_model, delta_t_, tau));
//         }
//       }

//       // GP factor
//       graph_.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t_, setting_.Qc_model));
//     }
//   }

// };

template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values sdf_mp_integration::GraphMaintainer<ROBOT, GP, SDFHandler, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::optimize(){

  gtsam::Values res  = gpmp2::optimize(graph_, init_values_, setting_);

  // save factor graph as graphviz dot file
  // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
  ofstream os("hsr_factor_graph.dot");
  graph_.saveGraph(os, res);

  // Also print out to console
  graph_.saveGraph(cout, res);

  return res;
};


template class sdf_mp_integration::GraphMaintainer<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>;
