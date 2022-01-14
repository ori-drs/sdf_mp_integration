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
 * \date    2020-09-09
 */
//----------------------------------------------------------------------


#ifndef SDF_MP_INTEGRATION_SDFHANDLER_H
#define SDF_MP_INTEGRATION_SDFHANDLER_H

#pragma once

#include <gpmp2/obstacle/SDFexception.h>
#include <gpmp2/config.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>

// #include "Fiesta.h"

// #include <voxblox_ros/ros_params.h>
// #include <voxblox_ros/esdf_server.h>

// #include <gpu_voxels_ros/gpu_voxels_server.h>
#include <gpu_voxels_ros/gpu_voxels_hsr_server.h>
#include <gpu_voxels_ros/live_composite_sdf.h>
#include <gpu_voxels_ros/single_composite_sdf.h>

#pragma once

// typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* FiestaPtr;
// typedef fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> FiestaClass;
// typedef std::shared_ptr<voxblox::EsdfMap> VoxbloxPtr;
// typedef voxblox::EsdfServer VoxbloxClass;
// typedef gpu_voxels_ros::GPUVoxelsServer* GPUVoxelsPtr;
// typedef gpu_voxels_ros::GPUVoxelsServer GPUVoxelsClass;
typedef gpu_voxels_ros::GPUVoxelsHSRServer* GPUVoxelsPtr;
typedef gpu_voxels_ros::GPUVoxelsHSRServer GPUVoxelsClass;
typedef gpu_voxels_ros::LiveCompositeSDF* LiveCompositeSDFPtr;
typedef gpu_voxels_ros::LiveCompositeSDF LiveCompositeSDFClass;
typedef gpu_voxels_ros::SingleCompositeSDF* SingleCompositeSDFPtr;
typedef gpu_voxels_ros::SingleCompositeSDF SingleCompositeSDFClass;

namespace sdf_mp_integration {

  template <typename SDFPACKAGEPTR>
  class SDFHandler {

    private:
      SDFPACKAGEPTR sdf_package_ptr_;
      size_t t_index_;
    public:
      
      //  constructor
      SDFHandler() {}      
      
      SDFHandler(SDFPACKAGEPTR sdf_package_ptr) {
        sdf_package_ptr_ = sdf_package_ptr;
      }

      SDFHandler(SDFPACKAGEPTR sdf_package_ptr, size_t t_index) {
        sdf_package_ptr_ = sdf_package_ptr;
        t_index_ = t_index;
      }

      ~SDFHandler() {}

      void print();

      /// give a point, search for signed distance field and (optional) gradient
      /// @param point query position
      /// @return signed distance
      double getSignedDistance(const gtsam::Point3& point) const ;

      /// give a point, search for signed distance field and (optional) gradient
      /// @param point query position
      /// @param g returned gradient reference
      /// @return signed distance
      double getSignedDistance(const gtsam::Point3& point, gtsam::Vector3& g) const ;
  };
} // sdf_mp_integration namespace

#endif