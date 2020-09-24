/*
 * fiesta_nodelet.h
 *
 *  Created on: 08/09/2020
 *  Author: Mark Finean
 */
#ifndef FIESTA_NODELET_SRC_FIESTA_NODELET_H_
#define FIESTA_NODELET_SRC_FIESTA_NODELET_H_
#include <nodelet/nodelet.h>
#include "Fiesta.h"

namespace fiesta
{
class FiestaNodelet : public nodelet::Nodelet
{
public:
    FiestaNodelet();
    ~FiestaNodelet();

    virtual void onInit();

    boost::shared_ptr<fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr>> inst_;

};
} // namespace fiesta

#endif /* FIESTA_NODELET_SRC_FIESTA_NODELET_H_ */
