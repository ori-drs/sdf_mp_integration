/**
 *  @file  SetHSRConf.h
 *  @brief Return known joint configuration for the HSR
 *  @author Mark Finean
 *  @date  Jan 07, 2021
 **/

#ifndef SDF_MP_INTEGRATION_SETHSRCONF_H
#define SDF_MP_INTEGRATION_SETHSRCONF_H

#include <gtsam/base/Vector.h>
#include <string>
#include <map>
#include <iostream>

namespace sdf_mp_integration {

    gtsam::Vector SetHSRConf(const std::string &conf_name);

} // sdf_mp_integration ns

#endif