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


#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <sdf_mp_integration/SetHSRConf.h>
#include <sdf_mp_integration/GenerateArm.h>

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

#include <sdf_mp_integration/ArmPose.h>
#include <sdf_mp_integration/WholeBodyPose.h>
#include <sdf_mp_integration/HeadDirection.h>

#include <sdf_mp_integration/utils/timing.h>


// To execute base commands on hsr
#include <tmc_omni_path_follower/PathFollowerAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <std_msgs/String.h>

#include <chrono>
#include <mutex>   

namespace sdf_mp_integration {

class PlanningServer{

    private:
      ros::NodeHandle node_;
      ros::Subscriber base_goal_sub_, arm_goal_sub_, full_goal_sub_, joint_sub_, odom_sub_;

      std::string base_goal_sub_topic_, arm_goal_sub_topic_, full_goal_sub_topic_, actual_base_sub_topic_;
      double resolution_;
      tf::TransformListener listener;
      sdf_mp_integration::SDFHandler<GPUVoxelsPtr>* sdf_handler_;
      ros::Publisher path_pub_, init_path_pub_, plan_msg_pub_, gaze_pub_, hsr_python_move_pub_;
      actionlib::SimpleActionClient<tmc_omni_path_follower::PathFollowerAction> execute_ac_ ;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_arm_ac_ ;
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> base_traj_ac_ ;

      int arm_dof_ = 5;

      int arm_lift_joint_ind = 1;  
      int arm_flex_joint_ind = 0;  
      int arm_roll_joint_ind = 2;  
      int wrist_flex_joint_ind = 11;  
      int wrist_roll_joint_ind = 12;  

      int odom_x_ind = 0;
      int odom_y_ind = 1;
      int odom_t_ind = 2;

      bool base_task_, arm_task_, full_task_;

      gtsam::Vector5 joint_state_, joint_v_state_;
      gtsam::Vector3 odom_state_, odom_v_state_;

      gpmp2::Pose2MobileVetLinArmModel arm_;
      std::vector<ros::Time> base_time_buffer_;
      std::vector<float> base_x_buffer_, base_y_buffer_, base_t_buffer_;

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
      ros::WallTime begin_t_;
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
      double last_traj_error;
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //  constructor
      PlanningServer() :  execute_ac_("path_follow_action", true), 
                          base_traj_ac_("/hsrb/omni_base_controller/follow_joint_trajectory", true), 
                          execute_arm_ac_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true) {}      
      
      PlanningServer(ros::NodeHandle node);

      ~PlanningServer() {}


      gtsam::Values getInitTrajectory(const gpmp2::Pose2Vector &start_pose, const gpmp2::Pose2Vector &end_pose);
      void reinitTrajectoryRemainder(gtsam::Values &traj_before, const size_t current_ind);
      void reinitTrajectory(gtsam::Values &traj);

      void clearBuffers();
      void recordExecutedTrajectory();
      void recordActualBase(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
      void odomStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
      void getCurrentPose(gpmp2::Pose2Vector &current_pose, gtsam::Vector &current_vel);

      void createSettings();
      void createSettings(float total_time, int total_time_step);
      void estimateSettings(const gpmp2::Pose2Vector& start_pose, const gpmp2::Pose2Vector& goal_pose);

      //For replanning
      void updateState(int idx);
      bool isTaskComplete();
      void replan(const ros::TimerEvent& /*event*/);
      void replan();



      //
      void moveToGo();
      void look(const float x, const float y, const float z, const std::string frame);
      void look(const gtsam::Values& traj, const size_t current_ind, const double t_look_ahead, const std::string frame);

      void armGoalCallback(const sdf_mp_integration::ArmPose::ConstPtr& msg);
      void baseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void fullGoalCallback(const sdf_mp_integration::WholeBodyPose::ConstPtr& msg);
      
      
      void visualiseTrajectory(const gtsam::Values& plan) const;
      void visualiseBasePlan(const gtsam::Values& plan) const;
      void visualiseInitialBasePlan(const gtsam::Values& plan) const;
      void executePathFollow(const gtsam::Values& plan);
      
      void executeTrajectory(const gtsam::Values& plan, const size_t current_ind = 0, const double t_delay = 0);
      void executeBaseTrajectory(const gtsam::Values& plan, const size_t current_ind = 0, const double t_delay = 0);
      void executeArmPlan(const gtsam::Values& plan, const size_t current_ind = 0, const double t_delay = 0);
      void executeFullPlan(const gtsam::Values& plan, const size_t current_ind = 0, const double t_delay = 0);

      void publishPlanMsg(const gtsam::Values& plan) const;

      void doneCb(const actionlib::SimpleClientGoalState& state, const tmc_omni_path_follower::PathFollowerResultConstPtr& result);
      void activeCb();
      void feedbackCb(const tmc_omni_path_follower::PathFollowerFeedbackConstPtr& feedback);

      template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
                class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
      void constructGraph(const ROBOT& arm,
                                    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
                                    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel);

      gtsam::Values optimize(const gtsam::Values& init_values);
      gtsam::Values manualOptimize(const gtsam::Values& init_values, bool iter_no_increase = true);
      gtsam::Values optimize(const gtsam::Values& init_values, double& final_err, bool iter_no_increase = true);

      // void armGoalCallback(const messagetype::ConstPtr& msg);
      // void fullGoalCallback(const messagetype::ConstPtr& msg);


};

} //ns
#endif
