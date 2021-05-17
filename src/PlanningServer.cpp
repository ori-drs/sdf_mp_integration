/**
 *  @file  PlanningServer.cpp
 *  @brief Planning server for the HSR
 *  @author Mark Finean
 *  @date  12 January, 2021
 **/

#include <sdf_mp_integration/PlanningServer.h>

sdf_mp_integration::PlanningServer::PlanningServer(ros::NodeHandle node) :  base_traj_ac_("/hsrb/omni_base_controller/follow_joint_trajectory", true), 
                                                                            execute_arm_ac_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true),
                                                                            head_traj_ac_("/hsrb/head_trajectory_controller/follow_joint_trajectory", true)
 {
                                                                            // execute_ac_("path_follow_action", true)   
    node_ = node;
    node_.param<std::string>("base_goal_sub_topic", base_goal_sub_topic_, "move_base_simple/goal");
    node_.param<std::string>("arm_goal_sub_topic", arm_goal_sub_topic_, "arm_goal");
    node_.param<std::string>("full_goal_sub_topic", full_goal_sub_topic_, "full_goal");
    node_.param<double>("resolution", resolution_, 0.05);
    delta_t_ = 0.5;

    base_task_ = false;
    arm_task_ = false;
    full_task_ = false;

    results_recorder_ = ResultsRecorder("/home/mark/iros2021", "");

    // Subscriptions
    base_goal_sub_ = node_.subscribe(base_goal_sub_topic_, 10, &PlanningServer::baseGoalCallback, this);
    arm_goal_sub_ = node_.subscribe(arm_goal_sub_topic_, 10, &PlanningServer::armGoalCallback, this);
    full_goal_sub_ = node_.subscribe(full_goal_sub_topic_, 10, &PlanningServer::fullGoalCallback, this);

    joint_sub_ = node_.subscribe("/hsrb/joint_states", 1, &PlanningServer::jointStateCallback, this);
    odom_sub_ = node_.subscribe("/hsrb/omni_base_controller/state", 1, &PlanningServer::odomStateCallback, this);

    path_pub_ = node_.advertise<nav_msgs::Path>("gpmp2_plan", 1000);
    init_path_pub_ = node_.advertise<nav_msgs::Path>("gpmp2_init_plan", 1000);
    plan_msg_pub_ = node_.advertise<sdf_mp_integration::GtsamValues>("gpmp2_results", 1);
    gaze_pub_ = node_.advertise<sdf_mp_integration::HeadDirection>("hsr_gaze_update", 1);
    hsr_python_move_pub_ = node_.advertise<std_msgs::String>("hsr_move_to_go", 1);

    arm_ = GenerateHSRArm();

    dh_vis_ = HSRVisualiser(node_);
    dh_vis_.setArm(arm_);

    // execute_ac_ = actionlib::SimpleActionClient<tmc_omni_path_follower::PathFollowerAction>("path_follow_action", true);
    // ROS_INFO("Waiting for action servers to start.");
    // execute_ac_.waitForServer();
    ROS_INFO("Waiting for action servers to start.");
    base_traj_ac_.waitForServer();
    ROS_INFO("execute_ac_ ready.");
    execute_arm_ac_.waitForServer();
    ROS_INFO("execute_arm_ac_ ready.");

    moveToGo();
    look(1.5,0.0,0.0, "base_footprint");
    ros::Duration(1).sleep();

    // // Start the mapping
    gpu_voxels_ptr_ = new gpu_voxels_ros::GPUVoxelsHSRServer(node_);
    sdf_handler_ = new sdf_mp_integration::SDFHandler<GPUVoxelsPtr>(gpu_voxels_ptr_);
    std::cout << "PlanningServer ready..." << std::endl;

    // Pause for mapping to take effect
    ros::Duration(2).sleep();

};

void sdf_mp_integration::PlanningServer::moveToGo(){
  std_msgs::String msg;
  msg.data = "";
  hsr_python_move_pub_.publish(msg);
  std::cout << "Move to go msg sent..." << std::endl;
}

// void sdf_mp_integration::PlanningServer::look(const float head_pan_joint, const float head_tilt_joint) {

//     // Look ahead
//     // sdf_mp_integration::HeadDirection head_msg;
//     // head_msg.pan_joint_state = head_pan_joint;
//     // head_msg.tilt_joint_state = head_tilt_joint;
//     // gaze_pub_.publish(head_msg);
//     this->executeHeadTrajectory(head_pan_joint, head_tilt_joint);

// }

void sdf_mp_integration::PlanningServer::look(const float x, const float y, const float z, const std::string frame) const{

    // Look ahead
    sdf_mp_integration::HeadDirection gaze_msg;
    gaze_msg.pt.x = x;
    gaze_msg.pt.y = y;
    gaze_msg.pt.z = z;
    gaze_msg.frame = frame;
    gaze_pub_.publish(gaze_msg);

}

void sdf_mp_integration::PlanningServer::look(const gtsam::Values& traj, const size_t current_ind, const double t_look_ahead, const std::string frame) const{
    
    gpmp2::Pose2Vector pose = traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', current_ind + ceil(t_look_ahead/delta_t_)));
    look(pose.pose().x(), pose.pose().y(), 0, frame);
}

gtsam::Values sdf_mp_integration::PlanningServer::getInitTrajectory(const gpmp2::Pose2Vector &start_pose, const gpmp2::Pose2Vector &end_pose){

    gtsam::Vector avg_vel = gtsam::Vector::Zero(arm_dof_+3);
    avg_vel[0] = (end_pose.pose().x()-start_pose.pose().x()) / (total_time_step_ * delta_t_);
    avg_vel[1] = (end_pose.pose().y()-start_pose.pose().y()) / (total_time_step_ * delta_t_);
    avg_vel[2] = (end_pose.pose().theta()-start_pose.pose().theta()) / (total_time_step_ * delta_t_);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      avg_vel[i+3] = (end_pose.configuration()[i] - start_pose.configuration()[i]) / (total_time_step_ * delta_t_);
    }

    gtsam::Values init_values;

    for (size_t i = 0; i < total_time_step_; i++)
    {
        gtsam::Symbol key_pos = gtsam::Symbol('x', i);
        gtsam::Symbol key_vel = gtsam::Symbol('v', i);
        
        // initialize as straight line in conf space
        gtsam::Vector conf = start_pose.configuration() * (total_time_step_-i)/total_time_step_ + end_pose.configuration() * i/total_time_step_;
        
        gtsam::Pose2 pose(start_pose.pose().x() * (total_time_step_-i)/total_time_step_ + end_pose.pose().x() * i/total_time_step_, 
                          start_pose.pose().y() * (total_time_step_-i)/total_time_step_ + end_pose.pose().y() * i/total_time_step_, 
                          start_pose.pose().theta() * (total_time_step_-i)/total_time_step_ + end_pose.pose().theta() * i/total_time_step_
        );

        // gtsam::insertPose2VectorInValues(key_pos, gpmp2::Pose2Vector(pose, conf), init_values);
        init_values.insert(key_pos, gpmp2::Pose2Vector(pose, conf));
        init_values.insert(key_vel, avg_vel);
    }
    return init_values;
}

void sdf_mp_integration::PlanningServer::reinitTrajectoryRemainder(gtsam::Values &traj_before, const size_t current_ind){

    gtsam::Symbol curr_key_pos = gtsam::Symbol('x', current_ind);
    gtsam::Symbol end_key_pos = gtsam::Symbol('x', total_time_step_ - 1);

    gpmp2::Pose2Vector curr_pose = traj_before.at<gpmp2::Pose2Vector>(curr_key_pos);

    gtsam::Vector avg_vel = gtsam::Vector::Zero(arm_dof_+3);
    float t_left = (total_time_step_ - current_ind) * delta_t_;
    
    avg_vel[0] = (goal_state_.pose().x() - curr_pose.pose().x()) / t_left;
    avg_vel[1] = (goal_state_.pose().y() - curr_pose.pose().y()) / t_left;
    avg_vel[2] = (goal_state_.pose().theta() - curr_pose.pose().theta()) / t_left;
    
    for (size_t i = 0; i < arm_dof_; i++)
    {
      avg_vel[i+3] = (goal_state_.configuration()[i] - curr_pose.configuration()[i]) / t_left;
    }

    for (size_t i = current_ind; i < total_time_step_; i++)
    {
        gtsam::Symbol key_pos = gtsam::Symbol('x', i);
        gtsam::Symbol key_vel = gtsam::Symbol('v', i);
        
        // initialize as straight line in conf space
        gtsam::Vector conf = curr_pose.configuration() * (total_time_step_-i)/total_time_step_ + goal_state_.configuration() * i/total_time_step_;
        
        gtsam::Pose2 pose(curr_pose.pose().x() * (total_time_step_-i)/total_time_step_ + goal_state_.pose().x() * i/total_time_step_, 
                          curr_pose.pose().y() * (total_time_step_-i)/total_time_step_ + goal_state_.pose().y() * i/total_time_step_, 
                          curr_pose.pose().theta() * (total_time_step_-i)/total_time_step_ + goal_state_.pose().theta() * i/total_time_step_
        );

        // gtsam::insertPose2VectorInValues(key_pos, gpmp2::Pose2Vector(pose, conf), init_values);
        traj_before.update(key_pos, gpmp2::Pose2Vector(pose, conf));
        traj_before.update(key_vel, avg_vel);
    }

}

void sdf_mp_integration::PlanningServer::reinitTrajectory(gtsam::Values &traj){
  
  gpmp2::Pose2Vector start_pose = traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', 0));
  traj = getInitTrajectory(start_pose, goal_state_);

}

void sdf_mp_integration::PlanningServer::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_[0] = msg->position[arm_lift_joint_ind];
    joint_state_[1] = msg->position[arm_flex_joint_ind];
    joint_state_[2] = msg->position[arm_roll_joint_ind];
    joint_state_[3] = msg->position[wrist_flex_joint_ind];
    joint_state_[4] = msg->position[wrist_roll_joint_ind];

    joint_v_state_[0] = msg->velocity[arm_lift_joint_ind];
    joint_v_state_[1] = msg->velocity[arm_flex_joint_ind];
    joint_v_state_[2] = msg->velocity[arm_roll_joint_ind];
    joint_v_state_[3] = msg->velocity[wrist_flex_joint_ind];
    joint_v_state_[4] = msg->velocity[wrist_roll_joint_ind];

    head_state_[0] = msg->position[pan_joint_ind];
    head_state_[1] = msg->position[tilt_joint_ind];
};

void sdf_mp_integration::PlanningServer::odomStateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    odom_state_[0] = msg->actual.positions[odom_x_ind];
    odom_state_[1] = msg->actual.positions[odom_y_ind];
    odom_state_[2] = msg->actual.positions[odom_t_ind];

    odom_v_state_[0] = msg->actual.velocities[odom_x_ind];
    odom_v_state_[1] = msg->actual.velocities[odom_y_ind];
    odom_v_state_[2] = msg->actual.velocities[odom_t_ind];

    tf::Transform transform;
    tf::Quaternion q;
    
    if( abs(odom_v_state_[0]) >= 0.05 || abs(odom_v_state_[1]) >=0.05 ){
      // last_yaw_ = 0.6*last_yaw_ + 0.4*atan2(std::ceil(odom_v_state_[1] * 10.0), std::ceil(odom_v_state_[0] * 10.0));
      last_yaw_ = atan2(std::ceil(odom_v_state_[1] * 10.0), std::ceil(odom_v_state_[0] * 10.0));
      std::cout << std::setprecision(2);
      // std::cout << "Proposed yaw update: " << atan2(std::ceil(odom_v_state_[1] * 10.0), std::ceil(odom_v_state_[0] * 10.0)) * 180.0 / 3.14 << std::endl;
      moving_ = true;
    }
    else{
      moving_ = false;
    }

    // std::cout << "Yaw: " << last_yaw_ << std::endl;
    q.setRPY(0,0,last_yaw_);
    q.normalize();
    transform.setOrigin( tf::Vector3(odom_state_[0], odom_state_[1], 0) );
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "travel"));

};

void sdf_mp_integration::PlanningServer::createSettings(float total_time, int total_time_step){
    total_time_step_ = total_time_step;
    total_time_ = total_time;
    node_.param<float>("epsilon", epsilon_, 0.5);
    // node_.param<float>("cost_sigma", cost_sigma_, 0.2);
    // node_.param<float>("epsilon", epsilon_, 0.2);
    node_.param<float>("cost_sigma", cost_sigma_, 0.05);
    node_.param<int>("obs_check_inter", obs_check_inter_, 4);
    node_.param<bool>("flag_pos_limit", flag_pos_limit_, false);
    node_.param<bool>("flag_vel_limit", flag_vel_limit_, true);

    // delta_t_ =  0.25;
    // delta_t_ =  total_time_/(total_time_step_-1);

    double pose_fix_sigma = 0.0001;
    double vel_fix_sigma = 0.0001; 
    gtsam::Vector joint_pos_limits_up(arm_dof_+3);
    gtsam::Vector joint_pos_limits_down(arm_dof_+3);
    gtsam::Vector pos_limit_thresh(arm_dof_+3);

    joint_pos_limits_down << -100, -100, -100,  0,     0,   -3.665,   -1.221,   -3.665;
    joint_pos_limits_up   << 100,   100,  100,  0.69,  2.617,        1.919,    1.919,    1.919;
    pos_limit_thresh = 0.001 * gtsam::Vector::Ones(arm_dof_+3);


    gtsam::Vector joint_vel_limit_vec_(arm_dof_+3);
    gtsam::Vector joint_vel_limit_thresh_(arm_dof_+3);

    // joint velocity limit param
    joint_vel_limit_vec_ << 0.2, 0.2, 0.5, 0.1, 0.3, 1.0, 1.0, 1.0;
    joint_vel_limit_thresh_ = 0.01 * gtsam::Vector::Ones(arm_dof_+3);
    


    // NOTE: The type here it bound to the dim... This needs changing for a different dof
    gtsam::Matrix8 Qc = 0.1 * gtsam::Matrix::Identity(arm_dof_+3,arm_dof_+3);


    setting_ = gpmp2::TrajOptimizerSetting(arm_dof_+3);
    // setting_.setGaussNewton();
    setting_.setLM();
    // setting_.setDogleg
    setting_.set_total_step(total_time_step_);
    setting_.set_total_time(total_time_);
    setting_.set_epsilon(epsilon_);
    setting_.set_cost_sigma(cost_sigma_);
    setting_.set_obs_check_inter(obs_check_inter_);
    setting_.set_conf_prior_model(pose_fix_sigma);
    setting_.set_vel_prior_model(vel_fix_sigma);
    setting_.set_Qc_model(Qc);
    // setting_.setVerbosityError();
    setting_.setVerbosityNone();
    setting_.set_flag_pos_limit(flag_pos_limit_);
    setting_.set_flag_vel_limit(flag_vel_limit_);

    setting_.set_joint_pos_limits_down(joint_pos_limits_up);
    setting_.set_joint_pos_limits_up(joint_pos_limits_down);
    setting_.set_pos_limit_thresh(pos_limit_thresh);

    setting_.set_vel_limits(joint_vel_limit_vec_);
    setting_.set_vel_limit_thresh(joint_vel_limit_thresh_);
    setting_.set_rel_thresh(1e-2);
    // setting_.set_max_iter(10);
}

void sdf_mp_integration::PlanningServer::createSettings(){

    node_.param<int>("total_time_step", total_time_step_, 51);
    node_.param<float>("total_time", total_time_, 20.0);
    createSettings(total_time_, total_time_step_);
}

void sdf_mp_integration::PlanningServer::getCurrentPose(gpmp2::Pose2Vector &current_pose, gtsam::Vector &current_vel){
    
    gtsam::Pose2 base_pose2(odom_state_[0], odom_state_[1], odom_state_[2]);

    current_pose = gpmp2::Pose2Vector(base_pose2, joint_state_);

    current_vel[0] = odom_v_state_[0];
    current_vel[1] = odom_v_state_[0];
    current_vel[2] = odom_v_state_[0];

    current_vel[3] = joint_v_state_[0];
    current_vel[4] = joint_v_state_[1];
    current_vel[5] = joint_v_state_[2];
    current_vel[6] = joint_v_state_[3];
    current_vel[7] = joint_v_state_[4];
}

void sdf_mp_integration::PlanningServer::updateState(int idx){
  // Adds prior factors to the factor graph to say where we are now

  gpmp2::Pose2Vector current_pose;
  gtsam::Vector current_vel(8);
  this->getCurrentPose(current_pose, current_vel);

  // estimate for pose and vel at given index
  graph_.add(gtsam::PriorFactor<gpmp2::Pose2Vector>(gtsam::Symbol('x', idx), current_pose, setting_.conf_prior_model));
  graph_.add(gtsam::PriorFactor<gtsam::Vector>(gtsam::Symbol('v', idx), current_vel, setting_.vel_prior_model));
}

bool sdf_mp_integration::PlanningServer::isTaskComplete(){
  // Check if we are within tolerance of the goal state


  if(base_task_){
    // std::cout << "xerr: " << abs(odom_state_[0] - goal_state_.pose().x())
    //           << "yerr: " <<  abs(odom_state_[1] - goal_state_.pose().y())
    //           << "terr: " <<  abs(odom_state_[2] - goal_state_.pose().theta()) << std::endl;

    return (abs(odom_state_[0] - goal_state_.pose().x()) <= 0.05 &&
    abs(odom_state_[1] - goal_state_.pose().y()) <= 0.05 &&
    abs(odom_state_[2] - goal_state_.pose().theta()) <= 0.1 // about 6 degrees
    );
  }
  else{
    std::cout << "xerr: " << abs(odom_state_[0] - goal_state_.pose().x())
              << "yerr: " <<  abs(odom_state_[1] - goal_state_.pose().y())
              << "terr: " <<  abs(odom_state_[2] - goal_state_.pose().theta()) 
              << "1err: " <<  abs(joint_state_[0] - goal_state_.configuration()[0])
              << "2err: " <<  abs(joint_state_[1] - goal_state_.configuration()[1])
              << "3err: " <<  abs(joint_state_[2] - goal_state_.configuration()[2])
              << "4err: " <<  abs(joint_state_[3] - goal_state_.configuration()[3]) 
              << "5err: " <<  abs(joint_state_[4] - goal_state_.configuration()[4]) << std::endl;

    return (abs(odom_state_[0] - goal_state_.pose().x()) <= 0.05 &&
    abs(odom_state_[1] - goal_state_.pose().y()) <= 0.05 &&
    abs(odom_state_[2] - goal_state_.pose().theta()) <= 0.1 && // about 2.5 degrees

    abs(joint_state_[0] - goal_state_.configuration()[0]) <= 0.02 &&
    abs(joint_state_[1] - goal_state_.configuration()[1]) <= 0.02 &&
    abs(joint_state_[2] - goal_state_.configuration()[2]) <= 0.02 &&
    abs(joint_state_[3] - goal_state_.configuration()[3]) <= 0.02 &&
    abs(joint_state_[4] - goal_state_.configuration()[4]) <= 0.02);
  }


};

bool sdf_mp_integration::PlanningServer::collisionCheck(const gtsam::Values &traj){

  bool isCollide = false;
  size_t interp_steps = 5;

  gtsam::Values interp_traj = gpmp2::interpolatePose2MobileArmTraj(traj, setting_.Qc_model, delta_t_, interp_steps, 0, total_time_step_-1);

  // TODO - Sort this horrible conversion out
  // Get a single obstacle factor from the graph to carry out the evaluation  
  // gtsam::FactorGraph<sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>>::sharedFactor fact_ptr = graph_.at(factor_index_dict_["obstacle"][0][0]);
  gtsam::NonlinearFactor::shared_ptr fact_ptr = graph_.at(factor_index_dict_["obstacle"][0][0]);
  sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>::shared_ptr conv_fact_ptr = boost::static_pointer_cast<sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>>(fact_ptr);

  // Loop through each time-step in the trajectory to check for a collision
  for (size_t i = 0; i < (total_time_step_-1)*(interp_steps+1) + 1; i++){   
    gpmp2::Pose2Vector pose = interp_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
    isCollide = isCollide || conv_fact_ptr->isInCollision(pose);
  }

  return isCollide;

}

void sdf_mp_integration::PlanningServer::replan(){
  
  if (!isTaskComplete())
  {

    double traj_error, new_traj_error, reinit_traj_error, old_err_improvement, reinit_err_improvement;
    // Calculate which index variable node we're at
    ros::WallTime current_t = ros::WallTime::now();
    ros::WallDuration dur = current_t - begin_t_;
    task_dur_ = current_t - task_callback_start_t_;


    gpmp2::Pose2Vector start_pose;
    gtsam::Vector start_vel(8);
    this->getCurrentPose(start_pose, start_vel);
    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);
    // dh_vis_.visualiseRobot(start_pose, 1);

    results_recorder_.recordActualTrajUpdate(task_dur_.toSec(), start_pose);

    // If less than 1s left, finish
    // if (dur.toSec() >= setting_.total_time - 1)
    // {
    //   return;
    // }

    double float_idx = dur.toSec()/delta_t_;
    int idx = round(float_idx);

    // Check if the path is still good
    traj_error = graph_.error(traj_res_);    
    // std::cout << "Last error: " << last_traj_error << "\t New error: " << traj_error << std::endl;
    // if(last_traj_error < 1.5 * traj_error && last_traj_error >= traj_error && traj_error < error_theshold && !collisionCheck(traj_res_)){
    if(last_traj_error < 1.5 * traj_error && last_traj_error >= traj_error && !collisionCheck(traj_res_) && !hasExecutionStopped()){
      // std::cout << "Using same trajectory."<< std::endl;
        sdf_mp_integration::Timer nbvTimer("PlanningNBVTimer");
        this->GetNBV(traj_res_, delta_t_, total_time_step_, idx + 2);
        nbvTimer.Stop();
      return;
    }

    // std::cout << "Trajectory no longer valid. Constructing new graph..."<< std::endl;

    // Create new graph from scratch



    double old_delta_t = delta_t_;
    double old_time_steps = total_time_step_;

    estimateSettings(start_pose, goal_state_);

    // New graph
    // sdf_mp_integration::Timer graphTimer("GraphConstruction");

    constructGraph<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm_, start_pose, start_vel, goal_state_, end_vel);
    // graphTimer.Stop();


    // Reset timings
    begin_t_ = ros::WallTime::now();
    last_idx_updated_ = 0;
    int iters;

    // initial values
    // std::cout << "Optimising for new graph" << std::endl;
    // sdf_mp_integration::Timer initTimer("initTimer");
    // gtsam::Values init_values_init = getInitTrajectory(start_pose, goal_state_);
    // initTimer.Stop();

    // std::cout << "refitTimer" << std::endl;

    // sdf_mp_integration::Timer refitTimer("refitTimer");
    gtsam::Values refit_values = sdf_mp_integration::refitPose2MobileArmTraj(traj_res_, start_pose, start_vel, setting_.Qc_model, old_delta_t, delta_t_, old_time_steps, total_time_step_, idx);
    // refitTimer.Stop();
    
    // std::cout << "initOptimiseTimer" << std::endl;
    // sdf_mp_integration::Timer initOptimiseTimer("initOptimiseTimer");
    // traj_res_ = this->optimize(init_values_init, traj_error, iters);
    // initOptimiseTimer.Stop();
    // std::cout << "init iters: " << iters << std::endl;

    // std::cout << "optimiseTimer" << std::endl;
    // sdf_mp_integration::Timer optimiseTimer("optimiseTimer");
    traj_res_ = this->optimize(refit_values, traj_error, iters);
    // optimiseTimer.Stop();
    // std::cout << "Refit iters: " << iters << std::endl;

    sdf_mp_integration::Timer nbvTimer("PlanningNBVTimer");
    this->GetNBV(traj_res_, delta_t_, total_time_step_, 2);
    nbvTimer.Stop();

    if(collisionCheck(traj_res_)){
      // std::cout << "Planned trajectory is in collision. Cancelling all current goals." << std::endl;
      cancelAllGoals();
      return;
    }

    // sdf_mp_integration::Timer interpVisualiseTimer("interpVisualiseTimer");
    // size_t interp_steps = 2;
    // gtsam::Values interp_traj = gpmp2::interpolatePose2MobileArmTraj(refit_values, setting_.Qc_model, delta_t_, interp_steps, 0, total_time_step_-1);
    // visualiseInitialBasePlan(interp_traj, (total_time_step_-1)*(interp_steps+1) + 1);
    visualiseInitialBasePlan(refit_values, total_time_step_);
    // interpVisualiseTimer.Stop();

    // sdf_mp_integration::Timer costsPrinterTimer("costsPrinterTimer");
    // printCosts(traj_res_);
    // costsPrinterTimer.Stop();
    // std::cout << "Finished printing costs" << std::endl;

    // Start timer and execute
    last_traj_error = traj_error;

    // publishPlanMsg(traj_res_);
    // std::cout << "Executing trajectory..." << std::endl;
    executeTrajectory(traj_res_, 0, 0.5);
    results_recorder_.recordTrajUpdate(task_dur_.toSec(), total_time_step_, traj_res_);

    // std::cout << "Interpolating and visualising..." << std::endl;
    // sdf_mp_integration::Timer interpVisualiseTimer2("interpVisualiseTimer2");
    // interp_traj = gpmp2::interpolatePose2MobileArmTraj(traj_res_, setting_.Qc_model, delta_t_, interp_steps, 0, total_time_step_-1);
    // visualiseTrajectory(interp_traj, (total_time_step_-1)*(interp_steps+1) + 1);
    visualiseTrajectory(traj_res_, total_time_step_);
    // interpVisualiseTimer2.Stop();


  } else{

    replan_timer_.stop();
    std::cout << "Finished re-planning - goal reached!" << std::endl;
    float map_coverage = gpu_voxels_ptr_->getPercentageMapExplored();
    ros::WallDuration task_dur = ros::WallTime::now() - task_callback_start_t_;

    std::cout << "Robot has observed " << map_coverage << "% of the map." << std::endl;
    std::cout << "The task took " << task_dur.toNSec() << "ns to complete." << std::endl;
    // results_recorder_.saveResults();
    // std::cout << "Results saved!" << std::endl;
    // look(1.0, 0, 0.0, "base_footprint");
  }

}

void sdf_mp_integration::PlanningServer::replan(const ros::TimerEvent& /*event*/){
  replan_mtx.lock();
  sdf_mp_integration::Timer replanTimer("replan");
  replan();
  replanTimer.Stop();
  // sdf_mp_integration::Timing::Print(std::cout);
  replan_mtx.unlock();
}

void sdf_mp_integration::PlanningServer::estimateSettings(const gpmp2::Pose2Vector& start_pose, const gpmp2::Pose2Vector& goal_pose){

  float est_traj_dist, est_traj_time;
  int est_steps;

  if(base_task_){
    est_traj_dist = sqrt(std::pow( goal_pose.pose().x() - start_pose.pose().x(), 2) + std::pow(goal_pose.pose().y() - start_pose.pose().y(), 2));
    est_traj_time = est_traj_dist / 0.15;
    est_steps = round(est_traj_time/delta_t_) + 1;  
    est_traj_time = est_steps * delta_t_;
  }
  else if(arm_task_){
    est_traj_time = 10; // TODO - need to automate this calculation
    est_steps = round(est_traj_time/delta_t_) + 1;
  }
  else if(full_task_){
    est_traj_dist = sqrt(std::pow( goal_pose.pose().x() - start_pose.pose().x(), 2) + std::pow(goal_pose.pose().y() - start_pose.pose().y(), 2));
    est_traj_time = est_traj_dist / 0.15;
    est_steps = round(est_traj_time/delta_t_) + 1;  
    est_traj_time = est_steps * delta_t_;
  }

  createSettings(est_traj_time, est_steps);

}

void sdf_mp_integration::PlanningServer::baseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    // this->TestNBV();

    task_callback_start_t_ = ros::WallTime::now();

    base_task_ = true;
    arm_task_ = false;
    full_task_ = false;

    // Look at the target location
    look(msg->pose.position.x, msg->pose.position.y, 0.0, "odom");
    ros::Duration(2).sleep();

    gpmp2::Pose2Vector start_pose;
    gtsam::Vector start_vel(8);
    this->getCurrentPose(start_pose, start_vel);
    results_recorder_.recordActualTrajUpdate(0, start_pose);

    // Get goal pose
    gtsam::Vector end_conf = sdf_mp_integration::SetHSRConf("go");
    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Quaternion q(msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double end_roll, end_pitch, end_yaw;
    m.getRPY(end_roll, end_pitch, end_yaw);
    gpmp2::Pose2Vector end_pose(gtsam::Pose2(msg->pose.position.x, msg->pose.position.y, end_yaw), end_conf);
    goal_state_ = end_pose;

    // Determine how long the trajectory should be and how it should be split up
    estimateSettings(start_pose, end_pose);

    // sdf_mp_integration::Timer graphTimer("GraphConstruction");

    constructGraph<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm_, start_pose, start_vel, end_pose, end_vel);
    // graphTimer.Stop();
    // printFactorTimeline();

    // initial values
    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose);
    visualiseInitialBasePlan(init_values, total_time_step_);

    if (replanning_)
    {
      last_idx_updated_ = 0;

      int iters;
      traj_res_ = this->optimize(init_values, last_traj_error, iters);
      results_recorder_.recordTrajUpdate(0.0, total_time_step_, traj_res_);
      // last_traj_error = graph_.error(traj_res_);
      // std::cout << "Initial iters: " << iters << std::endl;
      // sdf_mp_integration::Timer costsPrinterTimer("costsPrinterTimer");
      // printCosts(traj_res_);
      // costsPrinterTimer.Stop();

      publishPlanMsg(traj_res_);
      // Start timer and execute
      begin_t_ = ros::WallTime::now();

      sdf_mp_integration::Timer nbvTimer("PlanningNBVTimer");
      this->GetNBV(traj_res_, delta_t_, total_time_step_, 2);
      nbvTimer.Stop();

      executeBaseTrajectory(traj_res_, delta_t_, total_time_step_);
      visualiseBasePlan(traj_res_, total_time_step_);

      std::cout << "Executing. Now starting replan timer for every: " << round(1.0/0.1) << "Hz" << std::endl;
      replan_timer_ = node_.createTimer(ros::Duration(0.1), &sdf_mp_integration::PlanningServer::replan, this);
    }
    else{
      
      traj_res_ = this->optimize(init_values);
      executeBaseTrajectory(traj_res_, delta_t_, total_time_step_);
      visualiseBasePlan(traj_res_, total_time_step_);

      // for (size_t i = 0; i < trajectory_evolution_.size(); i++){
      //   visualiseBasePlan(trajectory_evolution_[i]);
      //   ros::Duration(0.2).sleep();
      // }

      publishPlanMsg(traj_res_);
    }
    

};

void sdf_mp_integration::PlanningServer::armGoalCallback(const sdf_mp_integration::ArmPose::ConstPtr& msg){

    task_callback_start_t_ = ros::WallTime::now();

    base_task_ = false;
    arm_task_ = true;
    full_task_ = false;

    gpmp2::Pose2Vector start_pose;
    gtsam::Vector start_vel(8);
    this->getCurrentPose(start_pose, start_vel);
    results_recorder_.recordActualTrajUpdate(0, start_pose);

    // Get goal pose
    gtsam::Vector end_conf(arm_dof_);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      end_conf[i] = msg->arm[i];
    }
    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);
    gpmp2::Pose2Vector end_pose(start_pose.pose(), end_conf); // Same starting base pose
    goal_state_ = end_pose;

    // settings
    // Determine how long the trajectory should be and how it should be split up
    estimateSettings(start_pose, end_pose);

    // initial values
    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose);
    visualiseInitialBasePlan(init_values, total_time_step_);

    sdf_mp_integration::Timer graphTimer("GraphConstruction");

    this->constructGraph<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm_, start_pose, start_vel, end_pose, end_vel);
    graphTimer.Stop();

    if (replanning_)
    {
      last_idx_updated_ = 0;

      traj_res_ = this->optimize(init_values);
      last_traj_error = graph_.error(traj_res_);
      publishPlanMsg(traj_res_);

      // Start timer and execute
      begin_t_ = ros::WallTime::now();
      executeArmPlan(traj_res_, delta_t_, total_time_step_);
      results_recorder_.recordTrajUpdate(0.0, total_time_step_, traj_res_);

      std::cout << "Executing. Now starting replan timer for every: " << round(1.0/0.2) << "Hz" << std::endl;
      replan_timer_ = node_.createTimer(ros::Duration(0.2), &sdf_mp_integration::PlanningServer::replan, this);
    }
    else{
      
      traj_res_ = this->optimize(init_values);
      executeArmPlan(traj_res_, delta_t_, total_time_step_);
      publishPlanMsg(traj_res_);
    }
};

void sdf_mp_integration::PlanningServer::fullGoalCallback(const sdf_mp_integration::WholeBodyPose::ConstPtr& msg){

    task_callback_start_t_ = ros::WallTime::now();
    base_task_ = false;
    arm_task_ = false;
    full_task_ = true;

    // Look at the target location
    look(msg->base.pose.position.x, msg->base.pose.position.y, 0.0, "odom");
    // look(msg->base.pose.position.x, msg->base.pose.position.y, 0.5, "odom");

    gpmp2::Pose2Vector start_pose;
    gtsam::Vector start_vel(8);
    this->getCurrentPose(start_pose, start_vel);
    results_recorder_.recordActualTrajUpdate(0, start_pose);


    // Get goal pose
    gtsam::Vector end_conf(arm_dof_);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      end_conf[i] = msg->arm[i];
    }

    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Quaternion q(msg->base.pose.orientation.x,
                    msg->base.pose.orientation.y,
                    msg->base.pose.orientation.z,
                    msg->base.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double end_roll, end_pitch, end_yaw;
    m.getRPY(end_roll, end_pitch, end_yaw);
    gpmp2::Pose2Vector end_pose(gtsam::Pose2(msg->base.pose.position.x, msg->base.pose.position.y, end_yaw), end_conf);
    goal_state_ = end_pose;

    // settings
    // Determine how long the trajectory should be and how it should be split up
    estimateSettings(start_pose, end_pose);

    // initial values

    sdf_mp_integration::Timer graphTimer("GraphConstruction");

    this->constructGraph<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm_, start_pose, start_vel, end_pose, end_vel);

    graphTimer.Stop();

    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose);
    visualiseInitialBasePlan(init_values, total_time_step_);

    if (replanning_)
    {
      last_idx_updated_ = 0;

      traj_res_ = this->optimize(init_values);
      last_traj_error = graph_.error(traj_res_);
      publishPlanMsg(traj_res_);

      // Start timer and execute
      begin_t_ = ros::WallTime::now();
      executeFullPlan(traj_res_, delta_t_, total_time_step_);
      visualiseBasePlan(traj_res_, total_time_step_);
      results_recorder_.recordTrajUpdate(0.0, total_time_step_, traj_res_);

      std::cout << "Executing. Now starting replan timer for every: " << round(1.0/0.2) << "Hz" << std::endl;
      replan_timer_ = node_.createTimer(ros::Duration(0.2), &sdf_mp_integration::PlanningServer::replan, this);
    }
    else{
      
      traj_res_ = this->optimize(init_values);
      executeFullPlan(traj_res_, delta_t_, total_time_step_);

      for (size_t i = 0; i < trajectory_evolution_.size(); i++){
        visualiseBasePlan(trajectory_evolution_[i], total_time_step_);
        ros::Duration(0.2).sleep();
      }

      publishPlanMsg(traj_res_);
    }
};

void sdf_mp_integration::PlanningServer::publishPlanMsg(const gtsam::Values& plan) const{
    sdf_mp_integration::GtsamValues plan_msg;
    plan_msg.delta_t = delta_t_;

    for (size_t i = 0; i < total_time_step_; i++)
    {
      gpmp2::Pose2Vector pose_x = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      gtsam::Vector pose_v = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));

      sdf_mp_integration::GtsamValue value_msg;
      
      value_msg.seq = i;
      value_msg.x.push_back(pose_x.pose().x());
      value_msg.x.push_back(pose_x.pose().y());
      value_msg.x.push_back(pose_x.pose().theta());
      
      value_msg.v.push_back(pose_v[0]);
      value_msg.v.push_back(pose_v[1]);
      value_msg.v.push_back(pose_v[2]);

      for (size_t i = 0; i < arm_dof_; i++)
      {
        value_msg.x.push_back(pose_x.configuration()[i]);
        value_msg.v.push_back(pose_v[i+3]);
      }

      plan_msg.values.push_back(value_msg);
    }
    plan_msg.header.stamp = ros::Time::now();
    plan_msg_pub_.publish(plan_msg);
};

void sdf_mp_integration::PlanningServer::visualiseInitialBasePlan(const gtsam::Values& plan, const size_t num_keys) const{
    nav_msgs::Path path;
    path.header.frame_id = "odom";


    for (size_t i = 0; i < num_keys; i++)
    {
      gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      tf::Matrix3x3 rot_mat;
      tf::Quaternion q;
      rot_mat.setEulerZYX(pose.pose().theta(), 0, 0);
	    rot_mat.getRotation(q);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.position.x = pose.pose().x();
      pose_msg.pose.position.y = pose.pose().y(); 
      pose_msg.pose.position.z = 0;
      pose_msg.pose.orientation.x = q[0];
      pose_msg.pose.orientation.y = q[1];
      pose_msg.pose.orientation.z = q[2];
      pose_msg.pose.orientation.w = q[3];

      path.poses.push_back(pose_msg);

    }
    
    init_path_pub_.publish(path);
};

void sdf_mp_integration::PlanningServer::visualiseTrajectory(const gtsam::Values& plan, const size_t num_keys) const{
      if(base_task_ || full_task_){
        visualiseBasePlan(plan, num_keys);
      }
      else{
        return;
      }
};

void sdf_mp_integration::PlanningServer::visualiseBasePlan(const gtsam::Values& plan, const size_t num_keys) const{
    nav_msgs::Path path;
    path.header.frame_id = "odom";

    for (size_t i = 0; i < num_keys; i++)
    {
      gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      tf::Matrix3x3 rot_mat;
      tf::Quaternion q;
      rot_mat.setEulerZYX(pose.pose().theta(), 0, 0);
	    rot_mat.getRotation(q);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.frame_id = "odom";
      pose_msg.pose.position.x = pose.pose().x();
      pose_msg.pose.position.y = pose.pose().y(); 
      pose_msg.pose.position.z = 0;
      pose_msg.pose.orientation.x = q[0];
      pose_msg.pose.orientation.y = q[1];
      pose_msg.pose.orientation.z = q[2];
      pose_msg.pose.orientation.w = q[3];

      path.poses.push_back(pose_msg);

    }
    
    path_pub_.publish(path);
};

bool sdf_mp_integration::PlanningServer::hasExecutionStopped() const{

  // PENDING=0, ACTIVE=1, PREEMPTED=2, SUCCEEDED=3, ABORTED=4, 
  // REJECTED=5, PREEMPTING=6, RECALLING=7, RECALLED=8, LOST=9

  if(base_task_){
    return (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) || 
            (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::REJECTED) || 
            (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED);
  }
  else if(arm_task_){
    return (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) || 
            (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::REJECTED) || 
            (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED);
  }
  else if(full_task_){
    return (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) || 
            (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::REJECTED) || 
            (base_traj_ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED) || 
            (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) || 
            (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::REJECTED) || 
            (execute_arm_ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED);
  }

}

void sdf_mp_integration::PlanningServer::cancelAllGoals(){
  // execute_ac_.cancelAllGoals();
  base_traj_ac_.cancelAllGoals();
  execute_arm_ac_.cancelAllGoals();
}

// void sdf_mp_integration::PlanningServer::executePathFollow(const gtsam::Values& plan) {
//     tmc_omni_path_follower::PathFollowerGoal path_goal;
//     path_goal.path_with_goal.header.frame_id = "odom";

//     for (size_t i = 0; i < total_time_step_; i++)
//     {
//       gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
//       tf::Matrix3x3 rot_mat;
//       tf::Quaternion q;
//       rot_mat.setEulerZYX(pose.pose().theta(), 0, 0);
// 	    rot_mat.getRotation(q);

//       geometry_msgs::PoseStamped pose_msg;
//       pose_msg.header.frame_id = "odom";
//       pose_msg.pose.position.x = pose.pose().x();
//       pose_msg.pose.position.y = pose.pose().y(); 
//       pose_msg.pose.position.z = 0;
//       pose_msg.pose.orientation.x = q[0];
//       pose_msg.pose.orientation.y = q[1];
//       pose_msg.pose.orientation.z = q[2];
//       pose_msg.pose.orientation.w = q[3];

//       path_goal.path_with_goal.poses.push_back(pose_msg);

//       if(i == total_time_step_ - 1){
//         path_goal.path_with_goal.goal = pose_msg.pose;
//       }
//     }

//     execute_ac_.sendGoal(path_goal);
    

// };

void sdf_mp_integration::PlanningServer::executeTrajectory(const gtsam::Values& plan, const size_t current_ind, const double t_delay) {

  size_t interp_steps = 2;

  // sdf_mp_integration::Timer interpExecutionTimer("interpExecutionTimer");
  gtsam::Values interp_traj = gpmp2::interpolatePose2MobileArmTraj(plan, setting_.Qc_model, delta_t_, interp_steps, 0, total_time_step_-1);
  // interpExecutionTimer.Stop();
 
  size_t num_keys = (total_time_step_-1)*(interp_steps+1) + 1;
  size_t start_ind = current_ind*(interp_steps+1);
  double new_delta_t = delta_t_/(interp_steps+1);

  if(base_task_){
    executeBaseTrajectory(interp_traj, new_delta_t, num_keys, start_ind, t_delay);
  }
  else if(arm_task_){
    executeArmPlan(interp_traj, new_delta_t, num_keys, start_ind, t_delay);
  }
  else if(full_task_){
    executeFullPlan(interp_traj, new_delta_t, num_keys, start_ind, t_delay);
  }

};

void sdf_mp_integration::PlanningServer::executeBaseTrajectory(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind, const double t_delay) {
    control_msgs::FollowJointTrajectoryGoal trajectory_goal;
    // trajectory_goal.trajectory.header.stamp = ros::Time::now();
    trajectory_goal.trajectory.header.frame_id = "odom";

    trajectory_goal.trajectory.joint_names.push_back("odom_x");      
    trajectory_goal.trajectory.joint_names.push_back("odom_y");      
    trajectory_goal.trajectory.joint_names.push_back("odom_t");      

    int delay_inds = ceil(t_delay/delta_t);

    trajectory_goal.trajectory.points.resize(num_keys - 1 - current_ind - delay_inds);
    // Don't include the first point which is current position

    size_t ctr = 0;
    // std::cout << "Num keys: " << num_keys << std::endl;
    for (size_t i = current_ind + delay_inds + 1; i < num_keys; i++){   
        // std::cout << i << std::endl;
        gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
        gtsam::Vector vel = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = {pose.pose().x(), pose.pose().y(), pose.pose().theta()};
        // pt.velocities = {vel[0], vel[1], vel[2]};
        // pt.accelerations;
        // pt.effort
        pt.time_from_start = ros::Duration((ctr + 1 + delay_inds) * delta_t); 

        // trajectory_goal.trajectory.points.append(pt);
        trajectory_goal.trajectory.points[ctr] = pt;
        ctr+=1;
    }

    base_traj_ac_.sendGoal(trajectory_goal);    

};

void sdf_mp_integration::PlanningServer::executeHeadTrajectory(const float head_pan_joint, const float head_tilt_joint, const size_t delay_inds) {
    control_msgs::FollowJointTrajectoryGoal trajectory_goal;
    trajectory_goal.trajectory.header.stamp = ros::Time::now();
    // trajectory_goal.trajectory.header.frame_id = "odom";

    trajectory_goal.trajectory.joint_names.push_back("head_pan_joint");      
    trajectory_goal.trajectory.joint_names.push_back("head_tilt_joint");      

    trajectory_goal.trajectory.points.resize(1);

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = {head_pan_joint, head_tilt_joint};
    pt.time_from_start = ros::Duration( delay_inds * delta_t_); 

    trajectory_goal.trajectory.points[0] = pt;

    head_traj_ac_.sendGoal(trajectory_goal);    

};

void sdf_mp_integration::PlanningServer::executeArmPlan(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind, const double t_delay) {
    control_msgs::FollowJointTrajectoryGoal arm_goal;

    arm_goal.trajectory.joint_names.push_back("arm_lift_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_roll_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_roll_joint");      

    int delay_inds = ceil(t_delay/delta_t);

    arm_goal.trajectory.points.resize(num_keys - 1 - current_ind - delay_inds);

    size_t ctr = 0;

    for (size_t i = current_ind + delay_inds + 1; i < num_keys; i++)
    {
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.time_from_start = ros::Duration((ctr + 1 + delay_inds) * delta_t); 

      gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      // gtsam::Vector v = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));
      for (size_t j = 0; j < arm_dof_; j++)
      {
        pt.positions.push_back(pose.configuration()[j]);
        // pt.velocities.push_back(v[j+3]);
      }

      arm_goal.trajectory.points[ctr] = pt;

      ctr+=1;
    }

    std::cout << "Sending arm goal" << std::endl;
    execute_arm_ac_.sendGoal(arm_goal);
    std::cout << "Arm goal sent" << std::endl;

};

void sdf_mp_integration::PlanningServer::executeFullPlan(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind, const double t_delay) {
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    control_msgs::FollowJointTrajectoryGoal path_goal;

    arm_goal.trajectory.joint_names.push_back("arm_lift_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_roll_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_roll_joint");      

    path_goal.trajectory.header.frame_id = "odom";
    path_goal.trajectory.joint_names.push_back("odom_x");      
    path_goal.trajectory.joint_names.push_back("odom_y");      
    path_goal.trajectory.joint_names.push_back("odom_t");      

    int delay_inds = ceil(t_delay/delta_t);

    path_goal.trajectory.points.resize(num_keys - 1 - current_ind - delay_inds);
    arm_goal.trajectory.points.resize(num_keys - 1 - current_ind - delay_inds);

    // std::cout << "Trajectories resized..." << std::endl;

    size_t ctr = 0;
    for (size_t i = current_ind + delay_inds + 1; i < num_keys; i++)
    {

      gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      // gtsam::Vector vel = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));

      // Base goal
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.time_from_start = ros::Duration((ctr + 1 + delay_inds) * delta_t); 
      pt.positions = {pose.pose().x(), pose.pose().y(), pose.pose().theta()};

      
      // Arm goal

      trajectory_msgs::JointTrajectoryPoint arm_pt;
      arm_pt.time_from_start = ros::Duration((ctr + 1 + delay_inds) * delta_t); 

      gtsam::Vector v = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));
      for (size_t j = 0; j < arm_dof_; j++)
      {
        arm_pt.positions.push_back(pose.configuration()[j]);
        // pt.velocities.push_back(vel[j+3]);
      }


      arm_goal.trajectory.points[ctr] = arm_pt;
      path_goal.trajectory.points[ctr] = pt;
      
      ctr+=1;
    }

    // std::cout << "Sending goals" << std::endl;
    execute_arm_ac_.sendGoal(arm_goal);
    base_traj_ac_.sendGoal(path_goal);

};

// TODO - fix this so that we dont manually have to changge the map dimensions
void sdf_mp_integration::PlanningServer::TestNBV(){
  std::cout << "sdf_mp_integration: Testing the NBV cone..." << std::endl;

  std::vector<robot::JointValueMap> robot_joints_vec;
  robot::JointValueMap joint_values;
  // joint_values["x_joint"] = odom_state_[0] + (0.5 * 448) * 0.025 ; 
  // joint_values["y_joint"] = odom_state_[1] + (0.5 * 448) * 0.025; 
  joint_values["x_joint"] = odom_state_[0]; 
  joint_values["y_joint"] = odom_state_[1]; 

  joint_values["theta_joint"] = odom_state_[2]; 

  joint_values["arm_lift_joint"] = joint_state_[0]; 
  joint_values["arm_flex_joint"] = joint_state_[1]; 
  joint_values["arm_roll_joint"] = joint_state_[2]; 
  joint_values["wrist_flex_joint"] = joint_state_[3]; 
  joint_values["wrist_roll_joint"] = joint_state_[4]; 

  joint_values["head_pan_joint"] = head_state_[0]; 
  joint_values["head_tilt_joint"] = head_state_[1]; 

  robot_joints_vec.push_back(joint_values);

  float nbv_joints[2] = {0,0};
  gpu_voxels_ptr_->GetNBV(robot_joints_vec, nbv_joints, 0);
  // this->look(nbv_joints[0], nbv_joints[1]);
  this->executeHeadTrajectory(nbv_joints[0], nbv_joints[1], 1);
}

void sdf_mp_integration::PlanningServer::GetNBV(const gtsam::Values& plan, const double delta_t, const size_t num_keys, const size_t current_ind){
  std::cout << "sdf_mp_integration: Getting the GetNBV..." << std::endl;

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

        joint_values["head_pan_joint"] = head_state_[0]; 
        joint_values["head_tilt_joint"] = head_state_[1]; 

        robot_joints_vec[i] = joint_values;

  }

  float nbv_joints[2] = {0,0};
  gpu_voxels_ptr_->GetNBV(robot_joints_vec, nbv_joints, current_ind);
  
  std::cout << "Requesting head move to \t pan: " << nbv_joints[0] << "\t tilt: " << nbv_joints[1] << std::endl;
  this->executeHeadTrajectory(nbv_joints[0], nbv_joints[1], 1);

  // this->look(nbv_joints[0], nbv_joints[1]);

}

void sdf_mp_integration::PlanningServer::printCosts(const gtsam::Values& traj){

  std:vector<std::string> factor_types = {"prior", "obstacle", "gp_obstacle", "gp", "pos", "vel"};

  std::cout << "--------------------------- Cost Timeline ----------------------------" << std::endl;
  
  // Print timeline
  std::cout << std::left << std::setfill(' ') << std::setw(12) << " "; // First column blank

  for (size_t t = 0; t < setting_.total_step; t++) {
    std::cout << std::left << std::setfill(' ') << std::setw(5) << t; //Print the timeindex
  }  
  std::cout << std::endl; 

  // Now print each row
  for (size_t i = 0; i < factor_types.size(); i++)
  {
    std::cout << std::fixed << std::left << std::setfill(' ') << std::setw(12) << factor_types[i];

    for (size_t t = 0; t < setting_.total_step; t++) {
      std::vector<size_t> inds_vec = factor_index_dict_[factor_types[i]][t];
            
      float cost = 0;

      for (size_t j = 0; j < inds_vec.size(); j++) {
        cost += graph_.at(inds_vec[j])->error(traj);
      }

      std::cout << std::fixed << std::left << std::setfill(' ') << std::setw(5) << std::setprecision(1) << cost;

    }  
    std::cout << std::endl; 

  }
}

void sdf_mp_integration::PlanningServer::printFactorTimeline(){

  std:vector<std::string> factor_types = {"prior", "obstacle", "gp_obstacle", "gp", "pos", "vel"};

  std::cout << "-------------------------- Factor Timeline ---------------------------" << std::endl;
  
  // Print timeline
  std::cout << std::left << std::setfill(' ') << std::setw(12) << " "; // First column blank

  for (size_t t = 0; t < setting_.total_step; t++) {
    std::cout << std::left << std::setfill(' ') << std::setw(3) << t; //Print the timeindex
  }  

  std::cout << std::endl; 

  // Now print each row
  for (size_t i = 0; i < factor_types.size(); i++)
  {
    std::cout << std::left << std::setfill(' ') << std::setw(12) << factor_types[i];

    for (size_t t = 0; t < setting_.total_step; t++) {
      std::vector<size_t> inds_vec = factor_index_dict_[factor_types[i]][t];
      std::cout << std::left << std::setfill(' ') << std::setw(3) << inds_vec.size();
    }  

    std::cout << std::endl; 
  }
  
}

template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void sdf_mp_integration::PlanningServer::constructGraph(
    const ROBOT& arm,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel) {

  // using namespace gtsam;

  graph_ = gtsam::NonlinearFactorGraph();

  // For bookkeeping storage of factors
  factor_index_dict_ = {
                          { "prior", std::vector<std::vector<size_t>>(setting_.total_step + 1) },
                          { "obstacle", std::vector<std::vector<size_t>>(setting_.total_step + 1) },
                          { "gp_obstacle", std::vector<std::vector<size_t>>(setting_.total_step + 1) },
                          { "pos", std::vector<std::vector<size_t>>(setting_.total_step + 1) },
                          { "vel", std::vector<std::vector<size_t>>(setting_.total_step + 1) },
                          { "gp", std::vector<std::vector<size_t>>(setting_.total_step + 1) }
                        };

  // fact_indices_ = std::vector<std::tuple< std::vector<size_t>, std::vector<size_t> > >(problem_setup_.total_time_step_ + 1);

  // GP interpolation setting
  const double delta_t = setting_.total_time / static_cast<double>(setting_.total_step);
  const double inter_dt = delta_t / static_cast<double>(setting_.obs_check_inter + 1);
  
  int factor_ind_counter = 0;

  for (size_t i = 0; i < setting_.total_step; i++) {
    gtsam::Key pose_key = gtsam::Symbol('x', i);
    gtsam::Key vel_key = gtsam::Symbol('v', i);

    std::vector<size_t> obs_fact_indices_in_timestep;
    std::vector<size_t> gp_obs_fact_indices_in_timestep;
    std::vector<size_t> gp_fact_indices_in_timestep;
    std::vector<size_t> prior_fact_indices_in_timestep;
    std::vector<size_t> vel_fact_indices_in_timestep;
    std::vector<size_t> pos_fact_indices_in_timestep;

    // start and end
    if (i == 0) {

      prior_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(gtsam::PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting_.conf_prior_model));
      factor_ind_counter += 1;

      prior_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(gtsam::PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting_.vel_prior_model));
      factor_ind_counter += 1;

    } else if (i == setting_.total_step - 1) {
      prior_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(gtsam::PriorFactor<typename ROBOT::Pose>(pose_key, end_conf, setting_.conf_prior_model));
      factor_ind_counter += 1;
      
      prior_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(gtsam::PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel, setting_.vel_prior_model));
      factor_ind_counter += 1;
    }

    if (setting_.flag_pos_limit) {
      // joint position limits
      pos_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(LIMIT_FACTOR_POS(pose_key, setting_.pos_limit_model, setting_.joint_pos_limits_down, 
          setting_.joint_pos_limits_up, setting_.pos_limit_thresh));
      factor_ind_counter += 1;
    }
    if (setting_.flag_vel_limit) {
      // velocity limits
      vel_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(LIMIT_FACTOR_VEL(vel_key, setting_.vel_limit_model, setting_.vel_limits, 
          setting_.vel_limit_thresh));
      factor_ind_counter += 1;
    }

    // non-interpolated cost factor
    obs_fact_indices_in_timestep.push_back(factor_ind_counter);
    graph_.add(OBS_FACTOR(pose_key, arm, *sdf_handler_, setting_.cost_sigma, setting_.epsilon));
    factor_ind_counter += 1;

    if (i > 0) {
      gtsam::Key last_pose_key = gtsam::Symbol('x', i-1);
      gtsam::Key last_vel_key = gtsam::Symbol('v', i-1);

      // interpolated cost factor
      if (setting_.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting_.obs_check_inter; j++) {
          const double tau = inter_dt * static_cast<double>(j);
          gp_obs_fact_indices_in_timestep.push_back(factor_ind_counter);
          graph_.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm, *sdf_handler_,
              setting_.cost_sigma, setting_.epsilon, setting_.Qc_model, delta_t, tau));
          factor_ind_counter += 1;
        }
      }

      // GP factor
      gp_fact_indices_in_timestep.push_back(factor_ind_counter);
      graph_.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t, setting_.Qc_model));
      factor_ind_counter += 1;
    }

    // fact_indices_[i]  = std::tuple< std::vector<size_t>, std::vector<size_t> >{obs_fact_indices_in_timestep , gp_obs_fact_indices_in_timestep};
    factor_index_dict_["prior"][i] = prior_fact_indices_in_timestep;
    factor_index_dict_["obstacle"][i] = obs_fact_indices_in_timestep;
    factor_index_dict_["gp_obstacle"][i] = gp_obs_fact_indices_in_timestep;
    factor_index_dict_["pos"][i] = pos_fact_indices_in_timestep;
    factor_index_dict_["vel"][i] = vel_fact_indices_in_timestep;
    factor_index_dict_["gp"][i] = gp_fact_indices_in_timestep;
  
  }

  // std::cout << "Graph constructed successfully" << std::endl;

}

gtsam::Values sdf_mp_integration::PlanningServer::optimize(const gtsam::Values& init_values){

  gtsam::Values res  = gpmp2::optimize(graph_, init_values, setting_);


  // save factor graph as graphviz dot file
  // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
  // ofstream os("hsr_factor_graph.dot");
  // graph_.saveGraph(os, res);

  // // Also print out to console
  // graph_.saveGraph(cout, res);

  return res;
};

gtsam::Values sdf_mp_integration::PlanningServer::optimize(const gtsam::Values& init_values, double& final_err, int& iters, bool iter_no_increase){

  using namespace std;
  using namespace gtsam;

  std::shared_ptr<gtsam::NonlinearOptimizer> opt;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> params;

  // init the params/opt and type specific settings
  if (setting_.opt_type == gpmp2::TrajOptimizerSetting::Dogleg) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new DoglegParams());
    // trust region ranage, 0.2 rad or meter, no whitenning, not sure make sense or not 
    dynamic_cast<DoglegParams*>(params.get())->setDeltaInitial(0.2);
  
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::LM) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new LevenbergMarquardtParams());
    dynamic_cast<LevenbergMarquardtParams*>(params.get())->setlambdaInitial(100.0);
  
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::GaussNewton) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new GaussNewtonParams());
  }

  // common settings
  params->setMaxIterations(setting_.max_iter);
  params->setRelativeErrorTol(setting_.rel_thresh);
  if (setting_.opt_verbosity >= gpmp2::TrajOptimizerSetting::Error)
    params->setVerbosity("ERROR");

  // optimizer
  if (setting_.opt_type == gpmp2::TrajOptimizerSetting::Dogleg) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new DoglegOptimizer(
      graph_, init_values, *(dynamic_cast<DoglegParams*>(params.get()))));
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::LM) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new LevenbergMarquardtOptimizer(
      graph_, init_values, *(dynamic_cast<LevenbergMarquardtParams*>(params.get()))));
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::GaussNewton) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new GaussNewtonOptimizer(
      graph_, init_values, *(dynamic_cast<GaussNewtonParams*>(params.get()))));
  }

  double currentError = opt->error();
  
  // check if we're already close enough
  if (currentError <= params->errorTol) {
    if (params->verbosity >= NonlinearOptimizerParams::ERROR){
      cout << "Exiting, as error = " << currentError << " < " << params->errorTol << endl;
    }
    final_err = opt->error();
    iters = opt->iterations();
    return opt->values();
  }

  // Maybe show output
  if (params->verbosity >= NonlinearOptimizerParams::ERROR){
    cout << "Initial error: " << currentError << endl;
  }

  // Return if we already have too many iterations
  if (opt->iterations() >= params->maxIterations) {
    if (params->verbosity >= NonlinearOptimizerParams::TERMINATION){
      cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    }
    final_err = opt->error();
    iters = opt->iterations();
    return opt->values();
  }

  Values last_values;

  // Iterative loop
  do {
    // iteration
    currentError = opt->error();
    // copy last values in case error increase
    if (iter_no_increase)
      last_values = opt->values();
    opt->iterate();
    // Maybe show output
    if (params->verbosity >= NonlinearOptimizerParams::ERROR){
      cout << "newError: " << opt->error() << endl;
    }
    // Store the result
  } while (opt->iterations() < params->maxIterations &&
      !checkConvergence(params->relativeErrorTol, params->absoluteErrorTol, params->errorTol,
          currentError, opt->error(), params->verbosity));

  // Printing if verbose
  if (params->verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    if (opt->iterations() >= params->maxIterations){
      cout << "Terminating because reached maximum iterations" << endl;
    }
  }

  // check whether values increase
  // if increase use last copied values
  if (opt->error() > currentError) {
    if (iter_no_increase) {
      if (params->verbosity >= NonlinearOptimizerParams::ERROR){
        cout << "Error increase, use last copied values" << endl;
      }    
      final_err = currentError;
      iters = opt->iterations();
      return last_values;

    } else {
      iters = opt->iterations();
      final_err = opt->error();
      return opt->values();
    }
  } else {
    iters = opt->iterations();
    final_err = opt->error();
    return opt->values();
  }

};

gtsam::Values sdf_mp_integration::PlanningServer::manualOptimize(const gtsam::Values& init_values, bool iter_no_increase){

  using namespace std;
  using namespace gtsam;

  std::shared_ptr<gtsam::NonlinearOptimizer> opt;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> params;

  trajectory_evolution_.clear();

  // init the params/opt and type specific settings
  if (setting_.opt_type == gpmp2::TrajOptimizerSetting::Dogleg) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new DoglegParams());
    // trust region ranage, 0.2 rad or meter, no whitenning, not sure make sense or not 
    dynamic_cast<DoglegParams*>(params.get())->setDeltaInitial(0.2);
  
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::LM) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new LevenbergMarquardtParams());
    dynamic_cast<LevenbergMarquardtParams*>(params.get())->setlambdaInitial(100.0);
  
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::GaussNewton) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(new GaussNewtonParams());
  }

  // common settings
  params->setMaxIterations(setting_.max_iter);
  params->setRelativeErrorTol(setting_.rel_thresh);
  if (setting_.opt_verbosity >= gpmp2::TrajOptimizerSetting::Error)
    params->setVerbosity("ERROR");

  // optimizer
  if (setting_.opt_type == gpmp2::TrajOptimizerSetting::Dogleg) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new DoglegOptimizer(
      graph_, init_values, *(dynamic_cast<DoglegParams*>(params.get()))));
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::LM) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new LevenbergMarquardtOptimizer(
      graph_, init_values, *(dynamic_cast<LevenbergMarquardtParams*>(params.get()))));
  } else if (setting_.opt_type == gpmp2::TrajOptimizerSetting::GaussNewton) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new GaussNewtonOptimizer(
      graph_, init_values, *(dynamic_cast<GaussNewtonParams*>(params.get()))));
  }

  double currentError = opt->error();
  
  // check if we're already close enough
  if (currentError <= params->errorTol) {
    if (params->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params->errorTol << endl;
    return opt->values();
  }

  // Maybe show output
  if (params->verbosity >= NonlinearOptimizerParams::ERROR)
    cout << "Initial error: " << currentError << endl;

  // Return if we already have too many iterations
  if (opt->iterations() >= params->maxIterations) {
    if (params->verbosity >= NonlinearOptimizerParams::TERMINATION)
      cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    return opt->values();
  }

  Values last_values;

  // Iterative loop
  do {
    // iteration
    currentError = opt->error();
    // copy last values in case error increase
    if (iter_no_increase)
      last_values = opt->values();
    opt->iterate();
    // Maybe show output
    if (params->verbosity >= NonlinearOptimizerParams::ERROR)
      cout << "newError: " << opt->error() << endl;

    // Store the result
    trajectory_evolution_.push_back(opt->values());

  } while (opt->iterations() < params->maxIterations &&
      !checkConvergence(params->relativeErrorTol, params->absoluteErrorTol, params->errorTol,
          currentError, opt->error(), params->verbosity));

  // Printing if verbose
  if (params->verbosity >= NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << opt->iterations() << " > " << params->maxIterations << endl;
    if (opt->iterations() >= params->maxIterations)
      cout << "Terminating because reached maximum iterations" << endl;
  }

  // check whether values increase
  // if increase use last copied values
  if (opt->error() > currentError) {
    if (iter_no_increase) {
      if (params->verbosity >= NonlinearOptimizerParams::ERROR)
        cout << "Error increase, use last copied values" << endl;
      trajectory_evolution_.push_back(last_values);
      return last_values;
    } else {
      return opt->values();
    }
  } else {
    return opt->values();
  }

};



// Called once when the goal completes
void sdf_mp_integration::PlanningServer::doneCb(const actionlib::SimpleClientGoalState& state,
            const tmc_omni_path_follower::PathFollowerResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void sdf_mp_integration::PlanningServer::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void sdf_mp_integration::PlanningServer::feedbackCb(const tmc_omni_path_follower::PathFollowerFeedbackConstPtr& feedback)
{
  ROS_INFO("Got progress of length %f", feedback->progress);
}
