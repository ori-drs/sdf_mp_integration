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
 * \author  Mark Finean (based on the GPMP2 Obstacle Factor GP by Jing Dong)
 * \date    2020-09-09
 */
//----------------------------------------------------------------------

using namespace std;
using namespace gtsam;


namespace sdf_mp_integration {

/* ************************************************************************** */
template <class SDFPACKAGEPTR, class ROBOT, class GPINTER>
gtsam::Vector ObstacleFactorGP<SDFPACKAGEPTR, ROBOT, GPINTER>::evaluateError(
    const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
    const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {

  const bool use_H = (H1 || H2 || H3 || H4);

  // if Jacobians used, initialize Jerr_conf as zeros
  // size: arm_nr_points_ * DOF
  Matrix Jerr_conf = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());


  // get conf by interpolation, except last pose
  typename Robot::Pose conf;
  Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
  if (use_H)
    conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2);
  else
    conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2);


  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;
  if (H1)
    robot_.sphereCenters(conf, sph_centers, J_px_jp);
  else
    robot_.sphereCenters(conf, sph_centers);


  // allocate cost vector
  Vector err(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

    const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

    if (H1) {
      Matrix13 Jerr_point;
      err(sph_idx) = sdf_mp_integration::hingeLossObstacleCost(sph_centers[sph_idx], sdf_handler_, total_eps, Jerr_point);

      // chain rules
      Jerr_conf.row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

    } else {
      err(sph_idx) = sdf_mp_integration::hingeLossObstacleCost(sph_centers[sph_idx], sdf_handler_, total_eps);
    }
  }

  // update jacobians
  if (use_H)
    GPBase::updatePoseJacobians(Jerr_conf, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2,
        H1, H2, H3, H4);

  return err;
}

}
