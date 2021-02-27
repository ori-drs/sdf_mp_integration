#include <sdf_mp_integration/utils/traj_utils.h>

    
gtsam::Values sdf_mp_integration::refitPose2MobileArmTraj(const gtsam::Values& input_traj, const gpmp2::Pose2Vector& start_pose, const gtsam::Vector& start_vel,
    const gtsam::SharedNoiseModel Qc_model, const double old_delta_t, const double new_delta_t, const size_t old_time_steps, const size_t new_time_steps, 
    const size_t start_index) {

    gtsam::Values new_traj;

    size_t result_index = 0;

    double r = new_delta_t/old_delta_t;

    // new_traj.insert(gtsam::Symbol('x', result_index), input_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', start_index)));
    // new_traj.insert(gtsam::Symbol('v', result_index), input_traj.at<gtsam::Vector>(gtsam::Symbol('v', start_index)));    
    new_traj.insert(gtsam::Symbol('x', result_index), start_pose);
    new_traj.insert(gtsam::Symbol('v', result_index), start_vel);
    result_index+=1;

    for (size_t i = 1; i < new_time_steps - 1 ; i++) {

        double frac_steps = r * i;
        size_t prev_ind =  start_index + floor(frac_steps);
        double tau = (frac_steps - floor(frac_steps)) * old_delta_t;

        if(prev_ind >= old_time_steps-1){
            new_traj.insert(gtsam::Symbol('x', result_index), input_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', old_time_steps-1)));
            new_traj.insert(gtsam::Symbol('v', result_index), input_traj.at<gtsam::Vector>(gtsam::Symbol('v', old_time_steps-1)));
        }
        else{
            gpmp2::Pose2Vector conf1 = input_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', prev_ind));
            gtsam::Vector vel1  = input_traj.at<gtsam::Vector>(gtsam::Symbol('v', prev_ind));
            gpmp2::Pose2Vector conf2 = input_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', prev_ind + 1));
            gtsam::Vector vel2  = input_traj.at<gtsam::Vector>(gtsam::Symbol('v', prev_ind + 1));

            gpmp2::GaussianProcessInterpolatorPose2Vector gp_inter(Qc_model, old_delta_t, tau);
            gpmp2::Pose2Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
            gtsam::Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
            new_traj.insert(gtsam::Symbol('x', result_index), conf);
            new_traj.insert(gtsam::Symbol('v', result_index), vel);
        }

        result_index+=1;
    }

    new_traj.insert(gtsam::Symbol('x', result_index), input_traj.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', old_time_steps-1)));
    new_traj.insert(gtsam::Symbol('v', result_index), input_traj.at<gtsam::Vector>(gtsam::Symbol('v', old_time_steps-1)));

    return new_traj;
}
