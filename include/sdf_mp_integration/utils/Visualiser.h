#ifndef SDF_MP_INTEGRATION_VISUALISER_H
#define SDF_MP_INTEGRATION_VISUALISER_H

#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>

template <class ModelType>
class Visualiser{
    private:
        ros::NodeHandle n_;
        ros::Publisher marker_pub_;
        std::vector<double> robot_spheres_radii_;
        size_t num_spheres_;
        ModelType arm_model_;

    public:

        Visualiser(){};

        Visualiser(ros::NodeHandle n);

        ~Visualiser(){};
        
        const void visualiseRobot(const gtsam::Vector &conf, const int id);
        const void visualiseRobot(const gpmp2::Pose2Vector &conf, const int id);

        const void visualiseRobotAxes(const gpmp2::Pose2Vector &conf, const int id);

        void setArm(ModelType& arm_model_);
};

typedef Visualiser<gpmp2::Pose2MobileVetLinArmModel> HSRVisualiser;

#endif
