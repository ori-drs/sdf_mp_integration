#include <sdf_mp_integration/HeadController.h>

// Constrain angles between -pi and pi
float constrainAngle(float x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::GetNextCameraPosition(const gtsam::Values& plan, float head_state[2], const double delta_t, const size_t num_keys, const size_t current_ind){
  
  switch (head_behaviour_)
  {
  case 0: // Constant
      return;
  case 1: // Panning
      pan();
      return;
  case 2: // 
      look(plan, current_ind, num_keys, 2.0, "odom");
      return;
  case 3:
      GetNBV(plan, head_state, delta_t, num_keys, current_ind);
      return;  
  default:
      std::cout << "PLEASE ENTER A VALID HEAD BEHAVIOUR" << std::endl;
      return;
  }

};

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::GetNBV(const gtsam::Values& plan, float head_state[2], const double delta_t, const size_t num_keys, const size_t current_ind){
//   std::cout << "sdf_mp_integration::HeadController::Getting the GetNBV. Ind:" << current_ind << "   num_keys:" << num_keys << std::endl;

  if(current_ind>=num_keys){
    return;
  }

  std::vector<robot::JointValueMap> robot_joints_vec(num_keys);

  // for (size_t i = current_ind; i < num_keys; i++)
  for (size_t i = 0; i < num_keys; i++)
  {
        gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));

        robot::JointValueMap joint_values;
        joint_values["x_joint"] = pose.pose().x(); 
        joint_values["y_joint"] = pose.pose().y(); 
        joint_values["theta_joint"] = pose.pose().theta(); 
        
        joint_values["arm_lift_joint"] = pose.configuration()[0]; 
        joint_values["arm_flex_joint"] = pose.configuration()[1]; 
        joint_values["arm_roll_joint"] = pose.configuration()[2]; 
        joint_values["wrist_flex_joint"] = pose.configuration()[3]; 
        joint_values["wrist_roll_joint"] = pose.configuration()[4]; 

        joint_values["head_pan_joint"] = head_state[0]; 
        joint_values["head_tilt_joint"] = head_state[1]; 

        robot_joints_vec[i] = joint_values;

  }

  float nbv_joints[2] = {0,0};
  
  gpu_voxels_ptr_->GetNBV(robot_joints_vec, nbv_joints, current_ind);
  
  //   std::cout << "Requesting head move to \t pan: " << nbv_joints[0] << "\t tilt: " << nbv_joints[1] << std::endl;
  this->executeHeadTrajectory(nbv_joints[0], nbv_joints[1], 1);

};

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::executeHeadTrajectory(const float head_pan_joint, const float head_tilt_joint, const size_t delay_inds) {
    control_msgs::FollowJointTrajectoryGoal trajectory_goal;
    trajectory_goal.trajectory.header.stamp = ros::Time::now();
    trajectory_goal.trajectory.joint_names.push_back("head_pan_joint");      
    trajectory_goal.trajectory.joint_names.push_back("head_tilt_joint");      

    trajectory_goal.trajectory.points.resize(1);

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = {head_pan_joint, head_tilt_joint};
    pt.time_from_start = ros::Duration( delay_inds * delta_t_); 

    trajectory_goal.trajectory.points[0] = pt;

    head_traj_ac_.sendGoal(trajectory_goal);    

};

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::look(const float x, const float y, const float z, const std::string frame) const{

    // Look ahead
    sdf_mp_integration::HeadDirection gaze_msg;
    gaze_msg.pt.x = x;
    gaze_msg.pt.y = y;
    gaze_msg.pt.z = z;
    gaze_msg.frame = frame;
    gaze_pub_.publish(gaze_msg);

}

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::look(const gtsam::Values& traj, const size_t current_ind,  const size_t num_keys, const double t_look_ahead, const std::string frame) const{
    
    size_t ind = std::min(  current_ind + ceil(t_look_ahead/delta_t_), (double) num_keys - 1);
    gpmp2::Pose2Vector pose = traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', ind));
    std::cout << "Looking to x: " << pose.pose().x() << "\t y: " <<  pose.pose().y() << std::endl;
    look(pose.pose().x(), pose.pose().y(), 0, frame);
}

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::pan() {
    
    if(pan_increasing_){
        float theta = constrainAngle(current_theta_ + delta_t_ * angular_v_);
        if(theta >= max_theta_){
            current_theta_ = max_theta_ - (theta - max_theta_);
            pan_increasing_ = false;
        }
        else{
            current_theta_ = theta;
        }
    }
    else{
        float theta = constrainAngle(current_theta_ - delta_t_ * angular_v_); 
        if(theta <= min_theta_){
            current_theta_ = min_theta_ - (theta - max_theta_);
            pan_increasing_ = true;
        }
        else{
            current_theta_ = theta;
        }
    }           

    this->executeHeadTrajectory(current_theta_, tilt_, 1);
}

template <typename SDFPACKAGEPTR>
void sdf_mp_integration::HeadController<SDFPACKAGEPTR>::lookForwards() {
    this->executeHeadTrajectory(0, 0, 1);
}

template class sdf_mp_integration::HeadController<gpu_voxels_ros::GPUVoxelsHSRServer*>;
template class sdf_mp_integration::HeadController<gpu_voxels_ros::LiveCompositeSDF*>;