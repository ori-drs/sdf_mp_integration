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
 * \author  Mark Finean (based on the GPMP2 hingeLossObstacleCost by Jing Dong)
 * \date    2020-09-09 
 */
//----------------------------------------------------------------------

using namespace std;
using namespace gtsam;


namespace sdf_mp_integration {

/* ************************************************************************** */
template <typename SDFPACKAGEPTR, typename ROBOT>
gtsam::Vector ObstacleFactor<SDFPACKAGEPTR, ROBOT>::evaluateError(
    const typename Robot::Pose& conf, boost::optional<gtsam::Matrix&> H1) const {

  // if Jacobians used, initialize as zeros
  // size: arm_nr_points_ * DOF
  if (H1) *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

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
      err(sph_idx) = sdf_mp_integration::hingeLossObstacleCost<SDFPACKAGEPTR>(sph_centers[sph_idx], sdf_handler_, total_eps, Jerr_point);

      // chain rules
      H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

    } else {
      err(sph_idx) = sdf_mp_integration::hingeLossObstacleCost<SDFPACKAGEPTR>(sph_centers[sph_idx], sdf_handler_, total_eps);
    }
  }

  return err;
}

template <typename SDFPACKAGEPTR, typename ROBOT>
gtsam::Vector ObstacleFactor<SDFPACKAGEPTR, ROBOT>::spheresInCollision(
    const typename Robot::Pose& conf) const {

  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  robot_.sphereCenters(conf, sph_centers);

  // allocate cost vector
  Vector spheres_in_collision(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

    const double sphere_radius = robot_.sphere_radius(sph_idx);
    if (sdf_mp_integration::hingeLossObstacleCost<SDFPACKAGEPTR>(sph_centers[sph_idx], sdf_handler_, sphere_radius)>0) {
      spheres_in_collision(sph_idx) = 1;
    }
    else
    {
      spheres_in_collision(sph_idx) = 0;
    }

  }

  return spheres_in_collision;
}

template <typename SDFPACKAGEPTR, typename ROBOT>
bool ObstacleFactor<SDFPACKAGEPTR, ROBOT>::isInCollision(
    const typename Robot::Pose& conf) const {

  return spheresInCollision(conf).sum() > 0;
}

}
