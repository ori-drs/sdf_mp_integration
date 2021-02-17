#ifndef SDF_MP_INTEGRATION_GENERATE_ARM_H
#define SDF_MP_INTEGRATION_GENERATE_ARM_H

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Arm.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

// For HSR Arm
#include <gpmp2/kinematics/Pose2MobileVetLinArm.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>


gpmp2::ArmModel GeneratePandaArm();
gpmp2::ArmModel GeneratePandaArm(const gtsam::Point3 &base_pos);

gpmp2::Pose2MobileVetLinArmModel GenerateHSRArm();
gpmp2::Pose2MobileVetLinArmModel GenerateHSRArm(const gtsam::Point3 &base_pos);

#endif