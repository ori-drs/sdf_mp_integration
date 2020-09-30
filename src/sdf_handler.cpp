#include <sample_nodelet/sdf_handler.h>

template <>
void sample_nodelet::SDFHandler<FiestaPtr>::print() {
  std::cout << "Test" << std::endl;
};

template <>
double sample_nodelet::SDFHandler<FiestaPtr>::getSignedDistance(const gtsam::Point3& point) const {
  return sdf_package_ptr_->GetDistance(point);
};

template <>
double sample_nodelet::SDFHandler<FiestaPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  return sdf_package_ptr_->GetDistanceAndGradient(point, g);
};


template <>
double sample_nodelet::SDFHandler<VoxbloxPtr>::getSignedDistance(const gtsam::Point3& point) const {
  double distance = 0.0;
  sdf_package_ptr_->getDistanceAtPosition(point, &distance);
  return distance;
};

template <>
double sample_nodelet::SDFHandler<VoxbloxPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  double distance = 0.0;
  sdf_package_ptr_->getDistanceAndGradientAtPosition(point, &distance, &g);
  return distance;
};

template <>
double sample_nodelet::SDFHandler<GPUVoxelsPtr>::getSignedDistance(const gtsam::Point3& point) const {
  double distance = 0.0;
  return sdf_package_ptr_->GetDistance(point);
};

template <>
double sample_nodelet::SDFHandler<GPUVoxelsPtr>::getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const {
  double distance = 0.0;
  return sdf_package_ptr_->GetDistanceAndGradient(point, g);
};
