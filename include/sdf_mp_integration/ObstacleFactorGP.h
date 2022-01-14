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


#pragma once

// #include <gpmp2/obstacle/SignedDistanceField.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <vector>

#include <gpmp2/obstacle/ObstacleCost.h>
#include <sdf_mp_integration/ObstacleCost.h>
#include <sdf_mp_integration/SDFHandler.h>

namespace sdf_mp_integration {

/**
 * binary factor for obstacle avoidance use GP interpolation, planar version
 * template robot model and GP interpolator version
 */

template <class SDFPACKAGEPTR, class ROBOT, class GPINTER>
class ObstacleFactorGP: public gtsam::NoiseModelFactor4<
    typename ROBOT::Pose, typename ROBOT::Velocity,
    typename ROBOT::Pose, typename ROBOT::Velocity> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;
  typedef typename Robot::Velocity Velocity;

private:
  // typedefs
  typedef ObstacleFactorGP This;
  typedef gtsam::NoiseModelFactor4<Pose, Velocity, Pose, Velocity> Base;
  typedef GPINTER GPBase;

  // GP interpolator
  GPBase GPbase_;

  // obstacle settings
  double epsilon_;      // global eps_ for hinge loss function

  // physical arm, with body sphere information
  const Robot& robot_;

  // a general SDF handle e.g. FIESTA, VoxBlox, GPU-Voxels
  const SDFHandler<SDFPACKAGEPTR>& sdf_handler_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  ObstacleFactorGP() : robot_(Robot()), sdf_handler_(SDFHandler<SDFPACKAGEPTR>()) {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param Qc_model   dim is equal to DOF
   * @param field      signed distance field
   * @param check_inter  how many points needed to be interpolated. 0 means no GP interpolation
   */
  ObstacleFactorGP(
      gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key, gtsam::Key vel2Key,
      const Robot& robot, const SDFHandler<SDFPACKAGEPTR>& sdf_handler, double cost_sigma,
      double epsilon, const gtsam::SharedNoiseModel& Qc_model, double delta_t, double tau) :

        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma),
        pose1Key, vel1Key, pose2Key, vel2Key), GPbase_(Qc_model, delta_t, tau),
        epsilon_(epsilon), robot_(robot), sdf_handler_(sdf_handler) {}

  ObstacleFactorGP(
      const gtsam::SharedNoiseModel& noiseModel, GPBase gpbase, gtsam::Key pose1Key, gtsam::Key vel1Key, gtsam::Key pose2Key, gtsam::Key vel2Key,
      const Robot& robot, const SDFHandler<SDFPACKAGEPTR>& sdf_handler, double epsilon) :

        Base(noiseModel, pose1Key, vel1Key, pose2Key, vel2Key), GPbase_(gpbase),
        epsilon_(epsilon), robot_(robot), sdf_handler_(sdf_handler) {}

  virtual ~ObstacleFactorGP() {}
  
  
  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(
      const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
      const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const ;


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter =gtsam:: DefaultKeyFormatter) const {
    std::cout << s << "ObstacleFactorGP :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor4",
        boost::serialization::base_object<Base>(*this));
  }
};

}

#include <sdf_mp_integration/ObstacleFactorGP-inl.h>

