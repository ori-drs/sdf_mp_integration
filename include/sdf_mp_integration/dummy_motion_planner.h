/*
 * dummy_motion_planner.h
 *
 *  Created on: 08/09/2020
 *  Author: Mark Finean
 */
#ifndef DUMMY_MOTION_PLANNER_SRC_DUMMY_MOTION_PLANNER_H_
#define DUMMY_MOTION_PLANNER_SRC_DUMMY_MOTION_PLANNER_H_

#include "Fiesta.h"
#include <Eigen/Eigen>


namespace dummy
{
    class DummyMotionPlanner
    {
    public:
        DummyMotionPlanner(fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* esdf_ptr);
        ~DummyMotionPlanner();

        double GetDistanceAndGradient(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);

        fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>* esdf_ptr_;
    };
} // namespace fiesta

#endif /* DUMMY_MOTION_PLANNER_SRC_DUMMY_MOTION_PLANNER_H_ */
