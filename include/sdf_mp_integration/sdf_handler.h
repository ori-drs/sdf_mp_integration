/**
 *  @file  sdf_handler.h
 *  @brief util functions for handling SDFs
 *  @author Mark Finean
 *  @date  09 September, 2020
 **/

#pragma once

#include <gpmp2/obstacle/SDFexception.h>
#include <gpmp2/config.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>

#include "Fiesta.h"

#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>

#include <gpu_voxels_ros/gpu_voxels_server.h>

#pragma once

typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* FiestaPtr;
typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> FiestaClass;
typedef std::shared_ptr<voxblox::EsdfMap> VoxbloxPtr;
typedef voxblox::EsdfServer VoxbloxClass;
typedef gpu_voxels_ros::GPUVoxelsServer* GPUVoxelsPtr;
typedef gpu_voxels_ros::GPUVoxelsServer GPUVoxelsClass;

namespace sdf_mp_integration {

  template <typename SDFPACKAGEPTR>
  class SDFHandler {

    private:
      SDFPACKAGEPTR sdf_package_ptr_;
    public:
      //  constructor
      SDFHandler() {}      
      
      SDFHandler(SDFPACKAGEPTR sdf_package_ptr) {
        sdf_package_ptr_ = sdf_package_ptr;
      }

      ~SDFHandler() {}

      void print();

      /// give a point, search for signed distance field and (optional) gradient
      /// @param point query position
      /// @return signed distance
      inline double getSignedDistance(const gtsam::Point3& point) const ;

      /// give a point, search for signed distance field and (optional) gradient
      /// @param point query position
      /// @param g returned gradient reference
      /// @return signed distance
      inline double getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const ;
  };
} // sdf_mp_integration namespace