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

#pragma once

#include <gpmp2/obstacle/SDFexception.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>
#include <sdf_mp_integration/SDFHandler.h>


namespace sdf_mp_integration {

  template <typename SDFPACKAGEPTR>
  inline double hingeLossObstacleCost(const gtsam::Point3& point, const sdf_mp_integration::SDFHandler<SDFPACKAGEPTR> sdf_handler,
      double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {

    gtsam::Vector3 field_gradient;
    double dist_signed;
    try {
      dist_signed = sdf_handler.getSignedDistance(point, field_gradient);
    } catch (gpmp2::SDFQueryOutOfRange&) {
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;
    } catch (std::out_of_range&) {
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;
    }
  
    if (dist_signed > eps) {
      // no error if far away
      if (H_point) *H_point = gtsam::Matrix13::Zero();
      return 0.0;

    } else {
      // outside but < eps or inside object
      if (H_point) *H_point = -field_gradient.transpose();
      return eps - dist_signed;
    }
  }
}

