/**
 *  @file  ObstacleCost.h
 *  @brief obstacle cost functions, implement hinge loss function
 *  @author Jing Dong
 *  @date  May 9, 2016
 **/

#pragma once

#include <gpmp2/obstacle/SDFexception.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <sdf_mp_integration/SDFHandler.h>


namespace sdf_mp_integration {

  // /// hinge loss obstacle cost function
  // inline double hingeLossObstacleCost(const gtsam::Point3& point, const sdf_mp_integration::SDFHandler<FiestaPtr> sdf_handler,
  //     double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {

  //   gtsam::Vector3 field_gradient;
  //   double dist_signed;
  //   try {
  //     dist_signed = sdf_handler.getSignedDistance(point, field_gradient);
  //   } catch (gpmp2::SDFQueryOutOfRange&) {
  //     //std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, "
  //     //    "assume zero obstacle cost." << std::endl;
  //     if (H_point) *H_point = gtsam::Matrix13::Zero();
  //     return 0.0;
  //   }

  //   if (dist_signed > eps) {
  //     // faraway no error
  //     if (H_point) *H_point = gtsam::Matrix13::Zero();
  //     return 0.0;

  //   } else {
  //     // outside but < eps or inside object
  //     if (H_point) *H_point = -field_gradient.transpose();
  //     return eps - dist_signed;
  //   }
  // }

  // inline double hingeLossObstacleCost(const gtsam::Point3& point, const sdf_mp_integration::SDFHandler<VoxbloxPtr> sdf_handler,
  //     double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {

  //   gtsam::Vector3 field_gradient;
  //   double dist_signed;
  //   try {
  //     dist_signed = sdf_handler.getSignedDistance(point, field_gradient);
  //   } catch (gpmp2::SDFQueryOutOfRange&) {
  //     //std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, "
  //     //    "assume zero obstacle cost." << std::endl;
  //     if (H_point) *H_point = gtsam::Matrix13::Zero();
  //     return 0.0;
  //   }

  //   if (dist_signed > eps) {
  //     // faraway no error
  //     if (H_point) *H_point = gtsam::Matrix13::Zero();
  //     return 0.0;

  //   } else {
  //     // outside but < eps or inside object
  //     if (H_point) *H_point = -field_gradient.transpose();
  //     return eps - dist_signed;
  //   }
  // }

  template <typename SDFPACKAGEPTR>
  inline double hingeLossObstacleCost(const gtsam::Point3& point, const sdf_mp_integration::SDFHandler<SDFPACKAGEPTR> sdf_handler,
      double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {

    gtsam::Vector3 field_gradient;
    double dist_signed;
    try {
      dist_signed = sdf_handler.getSignedDistance(point, field_gradient);
    } catch (gpmp2::SDFQueryOutOfRange&) {
      // std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, assume zero obstacle cost." << std::endl;
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;
    } catch (std::out_of_range&) {
      // std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, assume zero obstacle cost." << std::endl;
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;
    }
  
    if (dist_signed > eps) {
      // faraway no error
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;

    } else {
      // outside but < eps or inside object
      if (H_point) *H_point = -field_gradient.transpose();
      return eps - dist_signed;
    }
  }
}

