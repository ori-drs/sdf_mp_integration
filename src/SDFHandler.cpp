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
 * \author  Mark Finean
 * \date    2021-12-01
 */
//----------------------------------------------------------------------

#include <sdf_mp_integration/SDFHandler.h>

// template <>
// void sdf_mp_integration::SDFHandler<FiestaPtr>::print() {
//   std::cout << "Test" << std::endl;
// };

// template <>
// double sdf_mp_integration::SDFHandler<FiestaPtr>::getSignedDistance(const gtsam::Point3& point) const {
//   return sdf_package_ptr_->GetDistance(point);
// };

// template <>
// double sdf_mp_integration::SDFHandler<FiestaPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
//   return sdf_package_ptr_->GetDistanceAndGradient(point, g);
// };


// template <>
// double sdf_mp_integration::SDFHandler<VoxbloxPtr>::getSignedDistance(const gtsam::Point3& point) const {
//   double distance = 0.0;
//   sdf_package_ptr_->getDistanceAtPosition(point, &distance);
//   return distance;
// };

// template <>
// double sdf_mp_integration::SDFHandler<VoxbloxPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
//   double distance = 0.0;
//   sdf_package_ptr_->getDistanceAndGradientAtPosition(point, &distance, &g);
//   return distance;
// };

template <>
double sdf_mp_integration::SDFHandler<GPUVoxelsPtr>::getSignedDistance(const gtsam::Point3& point) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistance(point);
  return sdf_package_ptr_->GetTrilinearDistance(point);
};

template <>
double sdf_mp_integration::SDFHandler<GPUVoxelsPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistanceAndGradient(point, g);
  return sdf_package_ptr_->GetTrilinearDistanceAndGradient(point, g);
};

template <>
double sdf_mp_integration::SDFHandler<LiveCompositeSDFPtr>::getSignedDistance(const gtsam::Point3& point) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistance(point);
  return sdf_package_ptr_->GetTrilinearDistanceIndexed(point, t_index_);
};

template <>
double sdf_mp_integration::SDFHandler<LiveCompositeSDFPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistanceAndGradient(point, g);
  return sdf_package_ptr_->GetTrilinearDistanceAndGradientIndexed(point, g, t_index_);
};

template <>
double sdf_mp_integration::SDFHandler<SingleCompositeSDFPtr>::getSignedDistance(const gtsam::Point3& point) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistance(point);
  return sdf_package_ptr_->GetTrilinearDistance(point);
};

template <>
double sdf_mp_integration::SDFHandler<SingleCompositeSDFPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  // std::cout << "Distance: " << sdf_package_ptr_->GetDistance(point) << std::endl;
  // return sdf_package_ptr_->GetDistanceAndGradient(point, g);
  return sdf_package_ptr_->GetTrilinearDistanceAndGradient(point, g);
};

