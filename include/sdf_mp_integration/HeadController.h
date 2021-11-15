/**
 *  @file  HeadController.h
 *  @brief HeadController
 *  @author Mark Finean
 *  @date  17 May, 2021
 **/

#ifndef SDF_MP_INTEGRATION_HEADCONTROLLER_H
#define SDF_MP_INTEGRATION_HEADCONTROLLER_H

#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <gpu_voxels_ros/live_composite_sdf.h>
#include <gtsam/nonlinear/Values.h>
#include <sdf_mp_integration/HeadDirection.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/JointTrajectoryControllerState.h>
#include <gtsam/inference/Symbol.h>

#include <gpmp2/geometry/Pose2Vector.h>

namespace sdf_mp_integration {

  template <typename SDFPACKAGEPTR>
  class HeadController {

    private:
        SDFPACKAGEPTR gpu_voxels_ptr_;
        float delta_t_;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_traj_ac_ ;
        ros::NodeHandle node_;
        ros::Publisher gaze_pub_;
        int head_behaviour_; // 0 = constant, 1 = panning, 2 = lookahead, 3 = optimised
        int t_look_ahead_;

        bool pan_increasing_ = true;
        float current_theta_  = 0;
        float angular_v_ = 1.1;
        float min_theta_ = -3.5;
        float max_theta_ = 1.6;
        float tilt_ = -3.14/4;

    public:
      
        //  constructor
        HeadController() : head_traj_ac_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true) {}      
        
        HeadController(ros::NodeHandle node, SDFPACKAGEPTR gpu_voxels_ptr, float delta_t, int head_behaviour) : 
                    gpu_voxels_ptr_(gpu_voxels_ptr), head_traj_ac_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true), node_(node) , delta_t_(delta_t) , head_behaviour_(head_behaviour)
        {
                        gaze_pub_ = node_.advertise<sdf_mp_integration::HeadDirection>("hsr_gaze_update", 1);
        }      

        ~HeadController() {}

        void GetNextCameraPosition(const gtsam::Values& plan, float head_state[2], const double delta_t, const size_t num_keys, const size_t current_ind);
        void GetNBV(const gtsam::Values& plan, float head_state[2], const double delta_t, const size_t num_keys, const size_t current_ind);
        void executeHeadTrajectory(const float head_pan_joint, const float head_tilt_joint, const size_t current_ind);
        void look(const float x, const float y, const float z, const std::string frame) const;
        void look(const gtsam::Values& traj, const size_t current_ind,  const size_t num_keys, const double t_look_ahead, const std::string frame) const;
        void pan();
        void lookForwards();

  };
} // sdf_mp_integration namespace

#endif