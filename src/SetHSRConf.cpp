/**
 *  @file  SetHSRConf.cpp
 *  @brief Util for setting configs for the HSR
 *  @author Mark Finean
 *  @date  18 January, 2021
 **/

#include <sdf_mp_integration/SetHSRConf.h>

std::map<std::string, int> HSRMapConfToNum = {
    {"neutral", 1 },
    {"go", 2 },
    {"custom", 3 }
};

namespace sdf_mp_integration {

    gtsam::Vector SetHSRConf(const std::string &conf_name){
        gtsam::Vector conf(5);
        int conf_num = HSRMapConfToNum[conf_name];

        switch(conf_num) {
            case 1:
                conf << 0, 0, 0, -1.57, 0; break;
            case 2:
                conf << 0, 0, -1.57, -1.57, 0; break;
            case 3:
                // conf << 0.2, 1.0, -1.0, -1.0, 0; break;
                conf << 0.2, 1.0, -1.0, 1.57, -1; break;
            default: 
                std::cout << "Please enter a valid configuration name." << std::endl;
                break;
        }
        return conf;
    
    }; 

} // sdf_mp_integration ns