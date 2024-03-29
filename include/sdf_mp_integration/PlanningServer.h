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
 * \date    2020-09-09
 */
//----------------------------------------------------------------------

#ifndef SDF_MP_INTEGRATION_PLANNING_SERVER_H
#define SDF_MP_INTEGRATION_PLANNING_SERVER_H

#include <string.h>
#include <math.h> 

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <sdf_mp_integration/GtsamValues.h>
#include <sdf_mp_integration/GtsamValue.h>

#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


// #include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <gpu_voxels_ros/single_composite_sdf.h>
#include <sdf_mp_integration/SetHSRConf.h>
#include <sdf_mp_integration/GenerateArm.h>

#include <gpmp2/geometry/Pose2Vector.h>

#include <gpmp2/planner/TrajUtils.h>
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

#include <sdf_mp_integration/ArmPose.h>
#include <sdf_mp_integration/WholeBodyPose.h>

#include <sdf_mp_integration/utils/timing.h>
#include <sdf_mp_integration/utils/traj_utils.h>
#include <sdf_mp_integration/utils/Visualiser.h>
#include <sdf_mp_integration/ResultsRecorder.h>
#include <sdf_mp_integration/HeadController.h>


// To execute base commands on hsr
// #include <tmc_omni_path_follower/PathFollowerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <std_msgs/String.h>

#include <chrono>
#include <mutex>   

namespace sdf_mp_integration {

template <typename SDFPACKAGEPTR>
class PlanningServer{

    private:
      ros::NodeHandle node_;
      ros::Subscriber base_goal_sub_, arm_goal_sub_, full_goal_sub_, joint_sub_, odom_sub_;

      int head_behaviour_;
      std::string base_goal_sub_topic_, arm_goal_sub_topic_, full_goal_sub_topic_, actual_base_sub_topic_;
      double resolution_;
      tf::TransformListener listener;
      sdf_mp_integration::SDFHandler<SDFPACKAGEPTR>* sdf_handler_;
      ros::Publisher path_pub_, init_path_pub_, plan_msg_pub_, hsr_python_move_pub_;

      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_arm_ac_ ;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_traj_ac_ ;
      // actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_traj_ac_ ;

      ResultsRecorder results_recorder_;
      HSRVisualiser dh_vis_;
      sdf_mp_integration::HeadController<SDFPACKAGEPTR>* head_controller_;

      // GPUVoxelsPtr gpu_voxels_ptr_ = NULL; 
      // LiveCompositeSDFPtr gpu_voxels_live_composite_sdf_ptr_ = NULL;

      SDFPACKAGEPTR gpu_voxels_ptr_ = NULL; 

      std::vector<sdf_mp_integration::SDFHandler<LiveCompositeSDFPtr>*> composite_sdf_handlers_;
      size_t num_sdfs_;

      int arm_dof_ = 5;

      int arm_lift_joint_ind = 1;  
      int arm_flex_joint_ind = 0;  
      int arm_roll_joint_ind = 2;  
      int wrist_flex_joint_ind = 11;  
      int wrist_roll_joint_ind = 12;  

      int pan_joint_ind = 9;
      int tilt_joint_ind = 10;

      int odom_x_ind = 0;
      int odom_y_ind = 1;
      int odom_t_ind = 2;

      bool base_task_, arm_task_, full_task_;

      gtsam::Vector5 joint_state_, joint_v_state_;
      gtsam::Vector3 odom_state_, odom_v_state_;
      
      gpmp2::Pose2Vector current_pose_;
      gtsam::Vector current_vel_;
;

      float head_state_[2] = {};

      gpmp2::Pose2MobileVetLinArmModel arm_;
      std::vector<ros::Time> base_time_buffer_;
      std::vector<float> base_x_buffer_, base_y_buffer_, base_t_buffer_;

      std::map<std::string, std::vector< std::vector<size_t> > > factor_index_dict_;
      int total_time_step_;
      float total_time_;
      float epsilon_;
      float cost_sigma_;
      int obs_check_inter_;
      bool flag_pos_limit_, flag_vel_limit_;
      float delta_t_;
      gpmp2::TrajOptimizerSetting setting_;
      gtsam::NonlinearFactorGraph graph_;

      gpmp2::Pose2Vector goal_state_;
      ros::WallTime begin_t_, task_callback_start_t_;
      ros::WallDuration task_dur_, traj_dur_; 
      
      ros::Timer replan_timer_;
      std::vector<gtsam::Values> trajectory_evolution_;
      int last_idx_updated_;
      gtsam::Values traj_res_;

      std::mutex replan_mtx;
      // double traj_error_;
      bool replanning_ = true;
      tf::TransformBroadcaster br_;
      float last_yaw_ = 0;
      bool moving_ = false;
      double last_traj_error_;
      size_t goal_id_ = 0;

      int num_stops_ = 0;
      int num_stops_thresh_ = 2;
      int replan_attempts_ = 0;
      int replan_attempts_thresh_ = 3;

      // int stopped_
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //  constructor
      PlanningServer() :  base_traj_ac_("/hsrb/omni_base_controller/follow_joint_trajectory", true), 
                          execute_arm_ac_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true){}
                          // head_traj_ac_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true) {}      
      // execute_ac_("path_follow_action", true), 

      PlanningServer(ros::NodeHandle node);

      ~PlanningServer() {}

      void initGPUVoxels();

      gtsam::Values getInitTrajectory(const gpmp2::Pose2Vector &start_pose, const gpmp2::Pose2Vector &end_pose);
      gtsam::Values getRandomBaseInitTrajectory(const gpmp2::Pose2Vector &start_pose, const gpmp2::Pose2Vector &end_pose, const double max_variance);
      void reinitTrajectoryRemainder(gtsam::Values &traj_before, const size_t current_ind);
      void reinitTrajectory(gtsam::Values &traj);

      void clearBuffers();
      void recordExecutedTrajectory();
      void recordActualBase(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
      void odomStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
      void getCurrentPose(gpmp2::Pose2Vector &current_pose, gtsam::Vector &current_vel);
      void getCurrentStateUpdate();

      void createSettings();
      void createSettings(float total_time, int total_time_step);
      void estimateAndCreateSettings(const gpmp2::Pose2Vector& start_pose, const gpmp2::Pose2Vector& goal_pose);
      void estimateSettings(const gpmp2::Pose2Vector& start_pose, const gpmp2::Pose2Vector& goal_pose, float& est_traj_time, int &est_steps);

      //For replanning
      void updateState(int idx);
      bool isTaskComplete();
      void replan(const ros::TimerEvent& /*event*/);
      void replan();
      void finishTaskCleanup();

      bool isTrajectoryOnTime();
      bool isPathStillGood();
      bool replanTrajectory(gtsam::Values &refit_values, int idx);

      // void TestNBV();
      // void GetNBV(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind);

      //
      void moveToGo();
      // void look(const float head_pan_joint, const float head_tilt_joint);
      void look(const float x, const float y, const float z, const std::string frame) const;
      void look(const gtsam::Values& traj, const size_t current_ind, const double t_look_ahead, const std::string frame) const;

      void armGoalCallback(const sdf_mp_integration::ArmPose::ConstPtr& msg);
      void baseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void fullGoalCallback(const sdf_mp_integration::WholeBodyPose::ConstPtr& msg);
      
      
      void visualiseTrajectory(const gtsam::Values& plan, const size_t num_keys) const;
      void visualiseBasePlan(const gtsam::Values& plan, const size_t num_keys) const;
      void visualiseInitialBasePlan(const gtsam::Values& plan, const size_t num_keys) const;
      // void executePathFollow(const gtsam::Values& plan);

      bool hasExecutionStopped() const;
      void cancelAllGoals();
      void executeTrajectory(const gtsam::Values& plan, const size_t current_ind = 0, const double t_delay = 0);
      void executeBaseTrajectory(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind = 0, const double t_delay = 0);
      void executeArmPlan(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind = 0, const double t_delay = 0);
      void executeFullPlan(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind = 0, const double t_delay = 0);
      // void executeHeadTrajectory(const float head_pan_joint, const float head_tilt_joint, const size_t current_ind);

      void publishPlanMsg(const gtsam::Values& plan) const;

      // void doneCb(const actionlib::SimpleClientGoalState& state, const tmc_omni_path_follower::PathFollowerResultConstPtr& result);
      // void activeCb();
      // void feedbackCb(const tmc_omni_path_follower::PathFollowerFeedbackConstPtr& feedback);

      bool collisionCheck(const gtsam::Values &traj);
      
      template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
                class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
      void constructGraph(const ROBOT& arm,
                                    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
                                    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel);

      // template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
      //     class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
      // void constructIndexedGraph(const ROBOT& arm,
      //                     const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
      //                     const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel);


      void printCosts(const gtsam::Values& traj);
      void printFactorTimeline();
      gtsam::Values optimize(const gtsam::Values& init_values);
      gtsam::Values manualOptimize(const gtsam::Values& init_values, bool iter_no_increase = true);
      gtsam::Values optimize(const gtsam::Values& init_values, double& final_err, int& iters, bool iter_no_increase = true);

      // void armGoalCallback(const messagetype::ConstPtr& msg);
      // void fullGoalCallback(const messagetype::ConstPtr& msg);


};

} //ns
#endif
