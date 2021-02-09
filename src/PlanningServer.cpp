/**
 *  @file  PlanningServer.cpp
 *  @brief Planning server for the HSR
 *  @author Mark Finean
 *  @date  12 January, 2021
 **/

#include <sdf_mp_integration/PlanningServer.h>

sdf_mp_integration::PlanningServer::PlanningServer(ros::NodeHandle node) :  execute_ac_("path_follow_action", true), 
                                                                            base_traj_ac_("/hsrb/omni_base_controller/follow_joint_trajectory", true), 
                                                                            execute_arm_ac_("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true)      
 {
    node_ = node;
    node_.param<std::string>("base_goal_sub_topic", base_goal_sub_topic_, "move_base_simple/goal");
    node_.param<std::string>("arm_goal_sub_topic", arm_goal_sub_topic_, "arm_goal");
    node_.param<std::string>("full_goal_sub_topic", full_goal_sub_topic_, "full_goal");
    // node_.param<std::string>("actual_base_sub_topic", actual_base_sub_topic_, "/hsrb/omni_base_controller/state");
    node_.param<double>("resolution", resolution_, 0.05);
    
    // total_time_step_ = 21;

    // Subscriptions
    base_goal_sub_ = node_.subscribe(base_goal_sub_topic_, 10, &PlanningServer::baseGoalCallback, this);
    arm_goal_sub_ = node_.subscribe(arm_goal_sub_topic_, 10, &PlanningServer::armGoalCallback, this);
    full_goal_sub_ = node_.subscribe(full_goal_sub_topic_, 10, &PlanningServer::fullGoalCallback, this);

    // actual_base_sub_ = node_.subscribe(actual_base_sub_topic_, 1000, &PlanningServer::recordActualBase, this);
    joint_sub_ = node_.subscribe("/hsrb/joint_states", 1, &PlanningServer::jointStateCallback, this);

    path_pub_ = node_.advertise<nav_msgs::Path>("gpmp2_plan", 1000);
    init_path_pub_ = node_.advertise<nav_msgs::Path>("gpmp2_init_plan", 1000);
    plan_msg_pub = node_.advertise<sdf_mp_integration::GtsamValues>("gpmp2_results", 1);

    // execute_ac_ = actionlib::SimpleActionClient<tmc_omni_path_follower::PathFollowerAction>("path_follow_action", true);
    ROS_INFO("Waiting for action servers to start.");
    execute_ac_.waitForServer();
    ROS_INFO("Waiting for action servers to start.");
    base_traj_ac_.waitForServer();
    ROS_INFO("execute_ac_ ready.");
    execute_arm_ac_.waitForServer();
    ROS_INFO("execute_arm_ac_ ready.");

    // // Start the mapping
    GPUVoxelsPtr gpu_voxels_ptr; 
    gpu_voxels_ptr = new gpu_voxels_ros::GPUVoxelsHSRServer(node_);
    sdf_handler_ = new sdf_mp_integration::SDFHandler<GPUVoxelsPtr>(gpu_voxels_ptr);
    std::cout << "PlanningServer ready..." << std::endl;

    // Load the planning tool

    // setting_ = gpmp2::TrajOptimizerSetting(arm_dof_+3);
    // createSettings();

    // Pause for mapping to take effect
    ros::Duration(2).sleep();

};

gtsam::Values sdf_mp_integration::PlanningServer::getInitTrajectory(const gpmp2::Pose2Vector &start_pose, const gpmp2::Pose2Vector &end_pose, const float delta_t){

    gtsam::Vector avg_vel = gtsam::Vector::Zero(arm_dof_+3);
    avg_vel[0] = (end_pose.pose().x()-start_pose.pose().x()) / (total_time_step_ * delta_t);
    avg_vel[1] = (end_pose.pose().y()-start_pose.pose().y()) / (total_time_step_ * delta_t);
    avg_vel[2] = (end_pose.pose().theta()-start_pose.pose().theta()) / (total_time_step_ * delta_t);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      avg_vel[i+3] = (end_pose.configuration()[i] - start_pose.configuration()[i]) / (total_time_step_ * delta_t);
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

void sdf_mp_integration::PlanningServer::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state_[0] = msg->position[arm_lift_joint_ind];
    joint_state_[1] = msg->position[arm_flex_joint_ind];
    joint_state_[2] = msg->position[arm_roll_joint_ind];
    joint_state_[3] = msg->position[wrist_flex_joint_ind];
    joint_state_[4] = msg->position[wrist_roll_joint_ind];
};

void sdf_mp_integration::PlanningServer::createSettings(float total_time, int total_time_step){
    total_time_step_ = total_time_step;
    total_time_ = total_time;
    // node_.param<float>("epsilon", epsilon_, 0.5);
    // node_.param<float>("cost_sigma", cost_sigma_, 0.2);
    node_.param<float>("epsilon", epsilon_, 0.1);
    node_.param<float>("cost_sigma", cost_sigma_, 0.05);
    node_.param<int>("obs_check_inter", obs_check_inter_, 10);
    node_.param<bool>("flag_pos_limit", flag_pos_limit_, false);
    node_.param<bool>("flag_vel_limit", flag_vel_limit_, false);

    // delta_t_ =  0.25;
    // delta_t_ =  total_time_/(total_time_step_-1);

    double pose_fix_sigma = 0.0001;
    double vel_fix_sigma = 0.0001; 
    gtsam::Vector joint_pos_limits_up(arm_dof_+3);
    gtsam::Vector joint_pos_limits_down(arm_dof_+3);
    gtsam::Vector pos_limit_thresh(arm_dof_+3);

    joint_pos_limits_down << -100, -100, -100,  0,     -2.617,   -1.919,   -1.919,   -1.919;
    joint_pos_limits_up   << 100,   100,  100,  0.69,  0,        3.665,    1.221,    3.665;
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
    setting_.set_total_step(total_time_step_);
    setting_.set_total_time(total_time_);
    setting_.set_epsilon(epsilon_);
    setting_.set_cost_sigma(cost_sigma_);
    setting_.set_obs_check_inter(obs_check_inter_);
    setting_.set_conf_prior_model(pose_fix_sigma);
    setting_.set_vel_prior_model(vel_fix_sigma);
    setting_.set_Qc_model(Qc);
    setting_.setVerbosityError();
    setting_.set_flag_pos_limit(flag_pos_limit_);
    setting_.set_flag_vel_limit(flag_vel_limit_);

    setting_.set_joint_pos_limits_down(joint_pos_limits_up);
    setting_.set_joint_pos_limits_up(joint_pos_limits_down);
    setting_.set_pos_limit_thresh(pos_limit_thresh);

    setting_.set_vel_limits(joint_vel_limit_vec_);
    setting_.set_vel_limit_thresh(joint_vel_limit_thresh_);

    // setting_.set_max_iter(1);
}

void sdf_mp_integration::PlanningServer::createSettings(){

    node_.param<int>("total_time_step", total_time_step_, 51);
    node_.param<float>("total_time", total_time_, 20.0);
    createSettings(total_time_, total_time_step_);
}

void sdf_mp_integration::PlanningServer::baseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    // Get current base pose
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    // Set start pose
    gtsam::Vector start_conf(5);
    start_conf = joint_state_;
    
    // gtsam::Vector start_conf(dgpmp2::SetHSRConf("neutral"));
    gtsam::Vector start_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Matrix3x3 goal_m(transform.getRotation());
    double start_roll, start_pitch, start_yaw;
    goal_m.getRPY(start_roll, start_pitch, start_yaw);
    gtsam::Pose2 start_base_pose(transform.getOrigin().x(), transform.getOrigin().y(), start_yaw);
    gpmp2::Pose2Vector start_pose(start_base_pose, start_conf);


    // Get goal pose
    gtsam::Vector end_conf = dgpmp2::SetHSRConf("neutral");
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

    gpmp2::Pose2MobileVetLinArmModel arm = GenerateHSRArm();


    // Determine how long the trajectory should be and how it should be split up
    delta_t_ = 0.5;
    float est_traj_dist = sqrt(std::pow( msg->pose.position.x - transform.getOrigin().x(), 2) + std::pow(msg->pose.position.y - (float) transform.getOrigin().y(), 2));
    int est_traj_time = ceil( est_traj_dist / 0.15);
    // int est_steps = round(est_traj_time/0.25) + 1;
    int est_steps = round(est_traj_time/delta_t_) + 1;

    createSettings((float) est_traj_time, est_steps);

    // initial values
    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose, delta_t_);
    visualiseInitialBasePlan(init_values);

    // HSR
    gtsam::Values res = this->MarkTrajOptimize<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm, *sdf_handler_, start_pose, start_vel, end_pose, end_vel, init_values, setting_);

    // maintainer_ = sdf_mp_integration::GraphMaintainer<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
    //                                   sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
    //                                   sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
    //                                   gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm, *sdf_handler_, start_pose, start_vel, end_pose, end_vel, init_values, setting_);

    // this->constructGraph<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
    //                                   sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
    //                                   sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
    //                                   gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm, *sdf_handler_, start_pose, start_vel, end_pose, end_vel, setting_);

    // gtsam::Values res = maintainer_.optimize(init_values, setting_);
    
    // gtsam::Values res = this->optimize(init_values, setting_);


    std::cout << "Optimized" << std::endl;
    
    // Now visualise the base motion
    // publishPlanMsg(res);
    visualiseBasePlan(res);

    // clearBuffers();
    // executePathFollow(res);
    executeBaseTrajectory(res);

    // recordExecutedTrajectory();
    publishPlanMsg(res);
    // Now we need to compare actual vs desired


  // Create the factor graph problem 
};

void sdf_mp_integration::PlanningServer::armGoalCallback(const sdf_mp_integration::ArmPose::ConstPtr& msg){

    // Get goal pose
    gtsam::Vector end_conf(arm_dof_);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      end_conf[i] = msg->arm[i];
    }
    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);


    // Get current base pose
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }


    // Set start pose
    gtsam::Vector start_conf(5);
    start_conf = joint_state_;

    gtsam::Vector start_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Matrix3x3 goal_m(transform.getRotation());
    double start_roll, start_pitch, start_yaw;
    goal_m.getRPY(start_roll, start_pitch, start_yaw);
    gtsam::Pose2 start_base_pose(transform.getOrigin().x(), transform.getOrigin().y(), start_yaw);
    
    gpmp2::Pose2Vector start_pose(start_base_pose, start_conf);
    gpmp2::Pose2Vector end_pose(start_base_pose, end_conf); // Same starting base pose

    // settings

    gpmp2::Pose2MobileVetLinArmModel arm = GenerateHSRArm();

    double pose_fix_sigma = 0.0001;
    double vel_fix_sigma = 0.0001; 

    // NOTE: The type here it bound to the dim... This needs changing for a different dof
    gtsam::Matrix8 Qc = 1 * gtsam::Matrix::Identity(arm_dof_+3,arm_dof_+3);

    float delta_t =  10.0/(total_time_step_-1);

    gtsam::Vector joint_pos_limits_up(arm_dof_+3);
    gtsam::Vector joint_pos_limits_down(arm_dof_+3);
    gtsam::Vector pos_limit_thresh(arm_dof_+3);
    
    joint_pos_limits_down << -100, -100, -100,  0,     -2.617,   -1.919,   -1.919,   -1.919;
    joint_pos_limits_up   << 100,   100,  100,  0.69,  0,        3.665,    1.221,    3.665;
    pos_limit_thresh = 0.001 * gtsam::Vector::Ones(arm_dof_+3);

    gpmp2::TrajOptimizerSetting setting(arm_dof_+3);
    // setting.setGaussNewton();
    setting.setLM();
    setting.set_total_step(total_time_step_);
    setting.set_total_time(10);
    setting.set_epsilon(0.5);
    setting.set_cost_sigma(0.2);
    setting.set_obs_check_inter(1);
    setting.set_conf_prior_model(pose_fix_sigma);
    setting.set_vel_prior_model(vel_fix_sigma);
    setting.set_Qc_model(Qc);
    setting.setVerbosityError();
    setting.set_flag_pos_limit(false);
    setting.set_flag_vel_limit(false);
    setting.set_joint_pos_limits_down(joint_pos_limits_up);
    setting.set_joint_pos_limits_up(joint_pos_limits_down);
    setting.set_pos_limit_thresh(pos_limit_thresh);
    // setting.set_max_iter(1);

    // initial values
    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose, delta_t);

    // HSR
    gtsam::Values res = this->MarkTrajOptimize<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm, *sdf_handler_, start_pose, start_vel, end_pose, end_vel, init_values, setting);

    executeArmPlan(res, delta_t);
};

void sdf_mp_integration::PlanningServer::fullGoalCallback(const sdf_mp_integration::WholeBodyPose::ConstPtr& msg){

    // Get current base pose
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    double start_roll, start_pitch, start_yaw, end_roll, end_pitch, end_yaw;

    // Set start pose
    gtsam::Vector start_conf(5);
    start_conf = joint_state_;

    gtsam::Vector start_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Matrix3x3 goal_m(transform.getRotation());
    goal_m.getRPY(start_roll, start_pitch, start_yaw);
    gtsam::Pose2 start_base_pose(transform.getOrigin().x(), transform.getOrigin().y(), start_yaw);
    gpmp2::Pose2Vector start_pose(start_base_pose, start_conf);


    // Get goal pose
    // gtsam::Vector end_conf = dgpmp2::SetHSRConf("neutral");
    gtsam::Vector end_conf(arm_dof_);
    for (size_t i = 0; i < arm_dof_; i++)
    {
      end_conf[i] = msg->arm[i];
    }

    gtsam::Vector end_vel = gtsam::Vector::Zero(arm_dof_+3);
    tf::Quaternion q(msg->base.pose.orientation.x,
                    msg->base.pose.orientation.y,
                    msg->base.pose.orientation.z,
                    msg->base.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(end_roll, end_pitch, end_yaw);
    gpmp2::Pose2Vector end_pose(gtsam::Pose2(msg->base.pose.position.x, msg->base.pose.position.y, end_yaw), end_conf);
    
    // settings
    gpmp2::Pose2MobileVetLinArmModel arm = GenerateHSRArm();

    double pose_fix_sigma = 0.0001;
    double vel_fix_sigma = 0.0001; 

    // NOTE: The type here it bound to the dim... This needs changing for a different dof
    gtsam::Matrix8 Qc = 1 * gtsam::Matrix::Identity(arm_dof_+3,arm_dof_+3);

    float delta_t =  10.0/(total_time_step_-1);

    gtsam::Vector joint_pos_limits_up(arm_dof_+3);
    gtsam::Vector joint_pos_limits_down(arm_dof_+3);
    gtsam::Vector pos_limit_thresh(arm_dof_+3);
    
    // joint_pos_limits_down << -100, -100, -100,  0,     -2.617,   -1.919,   -1.919,   -1.919;
    // joint_pos_limits_up   << 100,   100,  100,  0.69,  0,        3.665,    1.221,    3.665;

    // Note: these have been reversed because the joints have been set up in negative
    joint_pos_limits_down << -100, -100, -100,  0,     0,   -3.665,   -1.221,   -3.665;
    joint_pos_limits_up   << 100,   100,  100,  0.69,  2.617, 1.919,    1.919,    1.919;
    
    pos_limit_thresh = 0.001 * gtsam::Vector::Ones(arm_dof_+3);

    gpmp2::TrajOptimizerSetting setting(arm_dof_+3);
    // setting.setGaussNewton();
    setting.setLM();
    setting.set_total_step(total_time_step_);
    setting.set_total_time(10);
    // setting.set_epsilon(0.5);
    setting.set_epsilon(0.2);
    setting.set_cost_sigma(0.2);
    setting.set_obs_check_inter(1);
    setting.set_conf_prior_model(pose_fix_sigma);
    setting.set_vel_prior_model(vel_fix_sigma);
    setting.set_Qc_model(Qc);
    setting.setVerbosityError();
    setting.set_flag_pos_limit(false);
    setting.set_flag_vel_limit(false);

    setting.set_joint_pos_limits_down(joint_pos_limits_up);
    setting.set_joint_pos_limits_up(joint_pos_limits_down);
    setting.set_pos_limit_thresh(pos_limit_thresh);

    // setting.set_max_iter(1);

    // initial values
    gtsam::Values init_values = getInitTrajectory(start_pose, end_pose, delta_t);

    visualiseInitialBasePlan(init_values);

    // HSR
    gtsam::Values res = this->MarkTrajOptimize<gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessPriorPose2Vector, sdf_mp_integration::SDFHandler<GPUVoxelsPtr>, 
                                      sdf_mp_integration::ObstacleFactor<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel>, 
                                      sdf_mp_integration::ObstacleFactorGP<GPUVoxelsPtr, gpmp2::Pose2MobileVetLinArmModel, gpmp2::GaussianProcessInterpolatorPose2Vector> , 
                                      gpmp2::JointLimitFactorPose2Vector, gpmp2::VelocityLimitFactorVector>(arm, *sdf_handler_, start_pose, start_vel, end_pose, end_vel, init_values, setting);
    // std::cout << "Finished planning!" << std::endl;

    // Now visualise the base motion
    publishPlanMsg(res);
    visualiseBasePlan(res);
    executePathFollow(res);
    executeFullPlan(res, delta_t);
    // std::cout << "Visualising base plan" << std::endl;

  // Create the factor graph problem 
};

void sdf_mp_integration::PlanningServer::publishPlanMsg(const gtsam::Values& plan){
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
    plan_msg_pub.publish(plan_msg);
};

void sdf_mp_integration::PlanningServer::visualiseInitialBasePlan(const gtsam::Values& plan){
    nav_msgs::Path path;
    path.header.frame_id = "odom";


    for (size_t i = 0; i < total_time_step_; i++)
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

void sdf_mp_integration::PlanningServer::visualiseBasePlan(const gtsam::Values& plan){
    nav_msgs::Path path;
    path.header.frame_id = "odom";

    // std::cout << "The poses are:" << std::endl;

    for (size_t i = 0; i < total_time_step_; i++)
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

      // std::cout << "\t" << i << ": " << pose.pose().x() << "\t" << pose.pose().y() << std::endl;
    }
    
    path_pub_.publish(path);
};

void sdf_mp_integration::PlanningServer::executePathFollow(const gtsam::Values& plan){
    tmc_omni_path_follower::PathFollowerGoal path_goal;
    path_goal.path_with_goal.header.frame_id = "odom";

    for (size_t i = 0; i < total_time_step_; i++)
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

      path_goal.path_with_goal.poses.push_back(pose_msg);

      if(i == total_time_step_ - 1){
        path_goal.path_with_goal.goal = pose_msg.pose;
      }
    }

    execute_ac_.sendGoal(path_goal);
    

};

void sdf_mp_integration::PlanningServer::executeBaseTrajectory(const gtsam::Values& plan){
    control_msgs::FollowJointTrajectoryGoal trajectory_goal;
    // trajectory_goal.trajectory.header.stamp = ros::Time::now();
    trajectory_goal.trajectory.header.frame_id = "odom";

    trajectory_goal.trajectory.joint_names.push_back("odom_x");      
    trajectory_goal.trajectory.joint_names.push_back("odom_y");      
    trajectory_goal.trajectory.joint_names.push_back("odom_t");      

    trajectory_goal.trajectory.points.resize(total_time_step_-1);
    // Don't include the first point which is current position
    for (size_t i = 1; i < total_time_step_; i++){   

        gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
        gtsam::Vector vel = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = {pose.pose().x(), pose.pose().y(), pose.pose().theta()};
        // pt.velocities = {vel[0], vel[1], vel[2]};
        // pt.accelerations;
        // pt.effort
        pt.time_from_start = ros::Duration(i * delta_t_); 

        // trajectory_goal.trajectory.points.append(pt);
        trajectory_goal.trajectory.points[i-1] = pt;
    }

    base_traj_ac_.sendGoal(trajectory_goal);    
};

void sdf_mp_integration::PlanningServer::executeArmPlan(const gtsam::Values& plan, const float delta_t){
    control_msgs::FollowJointTrajectoryGoal arm_goal;

    arm_goal.trajectory.joint_names.push_back("arm_lift_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_roll_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_roll_joint");      

    for (size_t i = 0; i < total_time_step_; i++)
    {
      trajectory_msgs::JointTrajectoryPoint pt;

      gpmp2::Pose2Vector pose = plan.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i));
      gtsam::Vector v = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));
      for (size_t j = 0; j < arm_dof_; j++)
      {
        pt.positions.push_back(pose.configuration()[j]);
        pt.velocities.push_back(v[j+3]);
      }

      // TODO - This is needed because the flexjoint is on wrong way
      pt.positions[1] = - pt.positions[1];
      pt.velocities[1] = - pt.velocities[1];

      pt.positions[2] = - pt.positions[2];
      pt.velocities[2] = - pt.velocities[2];

      pt.positions[3] = - pt.positions[3];
      pt.velocities[3] = - pt.velocities[3];

      pt.positions[4] = - pt.positions[4];
      pt.velocities[4] = - pt.velocities[4];


      // The last joint (5) is just wrong..

      pt.time_from_start = ros::Duration(i * delta_t);
      arm_goal.trajectory.points.push_back(pt);
    }

    std::cout << "Sending arm goal" << std::endl;
    execute_arm_ac_.sendGoal(arm_goal);
    std::cout << "Arm goal sent" << std::endl;

};

void sdf_mp_integration::PlanningServer::executeFullPlan(const gtsam::Values& plan, const float delta_t){
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    tmc_omni_path_follower::PathFollowerGoal path_goal;

    arm_goal.trajectory.joint_names.push_back("arm_lift_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("arm_roll_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_flex_joint");      
    arm_goal.trajectory.joint_names.push_back("wrist_roll_joint");      

    path_goal.path_with_goal.header.frame_id = "odom";

    for (size_t i = 0; i < total_time_step_; i++)
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

      path_goal.path_with_goal.poses.push_back(pose_msg);

      if(i == total_time_step_ - 1){
        path_goal.path_with_goal.goal = pose_msg.pose;
      }

      trajectory_msgs::JointTrajectoryPoint pt;

      gtsam::Vector v = plan.at<gtsam::Vector>(gtsam::Symbol('v', i));
      for (size_t j = 0; j < arm_dof_; j++)
      {
        pt.positions.push_back(pose.configuration()[j]);
        pt.velocities.push_back(v[j+3]);
      }

      // TODO - This is needed because the flexjoint is on wrong way
      pt.positions[1] = - pt.positions[1];
      pt.velocities[1] = - pt.velocities[1];

      pt.positions[2] = - pt.positions[2];
      pt.velocities[2] = - pt.velocities[2];

      pt.positions[3] = - pt.positions[3];
      pt.velocities[3] = - pt.velocities[3];

      pt.positions[4] = - pt.positions[4];
      pt.velocities[4] = - pt.velocities[4];

      pt.time_from_start = ros::Duration(i * delta_t);
      arm_goal.trajectory.points.push_back(pt);

    }

    std::cout << "Sending goals" << std::endl;
    execute_arm_ac_.sendGoal(arm_goal);
    execute_ac_.sendGoal(path_goal);

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

template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values sdf_mp_integration::PlanningServer::MarkTrajOptimize(
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
  
  for (size_t i = 0; i < setting.total_step; i++) {
    Key pose_key = Symbol('x', i);
    Key vel_key = Symbol('v', i);

    // start and end
    if (i == 0) {

      graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting.conf_prior_model));
      graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting.vel_prior_model));

    } else if (i == setting.total_step - 1) {
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
      graph.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t, setting.Qc_model));
    }
  }
  // graph.print();
  gtsam::Values res = gpmp2::optimize(graph, init_values, setting);
  // res.print();
  return res;
}

template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
gtsam::Values sdf_mp_integration::PlanningServer::constructGraph(
    const ROBOT& arm, const SDFHandler& sdf_handler,
    const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
    const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
    const gpmp2::TrajOptimizerSetting& setting) {

  using namespace gtsam;

  graph_ = gtsam::NonlinearFactorGraph();

  // GP interpolation setting
  const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
  const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);
  
  for (size_t i = 0; i < setting.total_step; i++) {
    Key pose_key = Symbol('x', i);
    Key vel_key = Symbol('v', i);

    // start and end
    if (i == 0) {

      graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting.conf_prior_model));
      graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting.vel_prior_model));

    } else if (i == setting.total_step - 1) {
      graph_.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf, setting.conf_prior_model));
      graph_.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel, setting.vel_prior_model));
    }

    if (setting.flag_pos_limit) {
      // joint position limits
      graph_.add(LIMIT_FACTOR_POS(pose_key, setting.pos_limit_model, setting.joint_pos_limits_down, 
          setting.joint_pos_limits_up, setting.pos_limit_thresh));
    }
    if (setting.flag_vel_limit) {
      // velocity limits
      graph_.add(LIMIT_FACTOR_VEL(vel_key, setting.vel_limit_model, setting.vel_limits, 
          setting.vel_limit_thresh));
    }

    // non-interpolated cost factor
    graph_.add(OBS_FACTOR(pose_key, arm, sdf_handler, setting.cost_sigma, setting.epsilon));

    if (i > 0) {
      Key last_pose_key = Symbol('x', i-1);
      Key last_vel_key = Symbol('v', i-1);

      // interpolated cost factor
      if (setting.obs_check_inter > 0) {
        for (size_t j = 1; j <= setting.obs_check_inter; j++) {
          const double tau = inter_dt * static_cast<double>(j);
          graph_.add(OBS_FACTOR_GP(last_pose_key, last_vel_key, pose_key, vel_key, arm, sdf_handler,
              setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
        }
      }

      // GP factor
      graph_.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t, setting.Qc_model));
    }
  }
}

gtsam::Values sdf_mp_integration::PlanningServer::optimize(const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting){

  gtsam::Values res  = gpmp2::optimize(graph_, init_values, setting);

  // save factor graph as graphviz dot file
  // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
  ofstream os("hsr_factor_graph.dot");
  graph_.saveGraph(os, res);

  // Also print out to console
  graph_.saveGraph(cout, res);

  return res;
};
