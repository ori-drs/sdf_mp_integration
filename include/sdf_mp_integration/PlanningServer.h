#ifndef SDF_MP_INTEGRATION_PLANNING_SERVER_H
#define SDF_MP_INTEGRATION_PLANNING_SERVER_H

#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <sdf_mp_integration/GtsamValues.h>
#include <sdf_mp_integration/GtsamValue.h>

#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>


#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <dgpmp2/SetHSRConf.h>
#include <dgpmp2/GenerateArm.h>

#include <gpmp2/geometry/Pose2Vector.h>

#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
// #include <gpmp2/planner/TrajUtils.h>

// #include <gtsam/nonlinear/ISAM2.h>
// #include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/slam/PriorFactor.h>
// #include <gtsam/nonlinear/DoglegOptimizer.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <sdf_mp_integration/SDFHandler.h>
#include <sdf_mp_integration/ObstacleFactor.h>
#include <sdf_mp_integration/ObstacleFactorGP.h>

#include <sdf_mp_integration/WholeBodyPose.h>



// To execute base commands on hsr
#include <tmc_omni_path_follower/PathFollowerAction.h>
#include <actionlib/client/simple_action_client.h>

namespace sdf_mp_integration {

class PlanningServer{

    private:
      ros::NodeHandle node_;
      ros::Subscriber base_goal_sub_, arm_goal_sub_, full_goal_sub_, joint_sub_;

      std::string base_goal_sub_topic_, arm_goal_sub_topic_, full_goal_sub_topic_;
      double resolution_;
      tf::TransformListener listener;
      sdf_mp_integration::SDFHandler<GPUVoxelsPtr>* sdf_handler_;
      ros::Publisher path_pub_, init_path_pub_, plan_msg_pub;
      size_t total_time_step_;
      actionlib::SimpleActionClient<tmc_omni_path_follower::PathFollowerAction> execute_ac_ ;
      int arm_dof = 5;

      int arm_lift_joint_ind = 1;  
      int arm_flex_joint_ind = 0;  
      int arm_roll_joint_ind = 2;  
      int wrist_flex_joint_ind = 11;  
      int wrist_roll_joint_ind = 12;  
      gtsam::Vector5 joint_state_;

    public:
      //  constructor
      PlanningServer() : execute_ac_("path_follow_action", true) {}      
      
      PlanningServer(ros::NodeHandle node);

      ~PlanningServer() {}

      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

      //
      void baseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void fullGoalCallback(const sdf_mp_integration::WholeBodyPose::ConstPtr& msg);
      void visualiseBasePlan(const gtsam::Values& plan);
      void visualiseInitialBasePlan(const gtsam::Values& plan);
      void executeBasePlan(const gtsam::Values& plan);
      void publishPlanMsg(const gtsam::Values& plan);

      void doneCb(const actionlib::SimpleClientGoalState& state, const tmc_omni_path_follower::PathFollowerResultConstPtr& result);
      void activeCb();
      void feedbackCb(const tmc_omni_path_follower::PathFollowerFeedbackConstPtr& feedback);

      template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
                class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
      gtsam::Values MarkTrajOptimize(const ROBOT& arm, const SDFHandler& sdf_handler,
                                                    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
                                                    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
                                                    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

      // void armGoalCallback(const messagetype::ConstPtr& msg);
      // void fullGoalCallback(const messagetype::ConstPtr& msg);


};

} //ns
#endif
