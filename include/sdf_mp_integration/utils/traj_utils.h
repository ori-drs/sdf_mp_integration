#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2.h>
#include <gpmp2/gp/GaussianProcessInterpolatorPose2Vector.h>

#include <gtsam/inference/Symbol.h>

#include <cmath>
#include <algorithm>

namespace sdf_mp_integration {

    gtsam::Values refitPose2MobileArmTraj(const gtsam::Values& input_traj,
        const gtsam::SharedNoiseModel Qc_model, const double old_delta_t, const double new_delta_t, const size_t old_time_steps, const size_t new_time_steps, 
        const size_t start_index);

}