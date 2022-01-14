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
 * \date    2021-18-01
 */
//----------------------------------------------------------------------

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