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

