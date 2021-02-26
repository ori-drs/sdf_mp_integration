/**
 *  @file  ObstacleFactor.h
 *  @brief Obstacle avoidance cost factor template 
 *  @author Mark Finean
 *  @date  09 September, 2020
 **/


#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <gpmp2/obstacle/ObstacleCost.h>
#include <sdf_mp_integration/ObstacleCost.h>

#include <iostream>
#include <vector>
#include <sdf_mp_integration/SDFHandler.h>

using namespace sdf_mp_integration;

namespace sdf_mp_integration {

/**
 * unary factor for obstacle avoidance
 * template robot model version
 */
template <typename SDFPACKAGEPTR, typename ROBOT>
class ObstacleFactor: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

private:
  // typedefs
  typedef ObstacleFactor This;
  typedef gtsam::NoiseModelFactor1<Pose> Base;

  // obstacle cost settings
  double epsilon_;      // distance from object that start non-zero cost

  // arm: planar one, all alpha = 0
  const Robot& robot_;

  // a general SDF handle e.g. FIESTA, VoxBlox, GPU-Voxels
  const SDFHandler<SDFPACKAGEPTR>& sdf_handler_;


public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  ObstacleFactor() : robot_(Robot()), sdf_handler_(SDFHandler<SDFPACKAGEPTR>()) {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   */
  // template <SDFPACKAGEPTR, ROBOT>
  ObstacleFactor(gtsam::Key poseKey, const Robot& robot,
      const SDFHandler<SDFPACKAGEPTR>& sdf_handler, double cost_sigma, double epsilon) :
        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(), cost_sigma), poseKey),
        epsilon_(epsilon), robot_(robot), sdf_handler_(sdf_handler) {}

  // template <SDFPACKAGEPTR, ROBOT>
  ObstacleFactor(const gtsam::SharedNoiseModel& noiseModel, gtsam::Key poseKey, const Robot& robot,
      const SDFHandler<SDFPACKAGEPTR>& sdf_handler, double epsilon) :
        Base(noiseModel, poseKey),
        epsilon_(epsilon), robot_(robot), sdf_handler_(sdf_handler) {}

  virtual ~ObstacleFactor() {}


  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(const typename Robot::Pose& conf,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const ;

  gtsam::Vector spheresInCollision(const typename Robot::Pose& conf) const ;

  bool isInCollision(const typename Robot::Pose& conf) const ;

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "ObstacleFactor :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
  }
};

}

#include <sdf_mp_integration/ObstacleFactor-inl.h>


