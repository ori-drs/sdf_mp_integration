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

#ifndef SDF_MP_INTEGRATION_RESULTSRECORDER_H
#define SDF_MP_INTEGRATION_RESULTSRECORDER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gpmp2/geometry/Pose2Vector.h>

class ResultsRecorder
{
    private:
        std::string foldername_;
        std::string file_prefix_;
        size_t dof_;

        std::vector<std::tuple<double, size_t, gtsam::Values>> traj_updates_;
        std::vector<std::tuple<double, gpmp2::Pose2Vector>> actual_traj_;


    public:
        ResultsRecorder(){};
        ResultsRecorder(const std::string folder, const std::string file_prefix);

        virtual ~ResultsRecorder(){};

        void recordTrajUpdate(const double t, const size_t num_keys, const gtsam::Values& trajectory);
        
        void recordActualTrajUpdate(const double t, const gpmp2::Pose2Vector& current_pose);

        const void saveResults();

        const void createSaveDir();
};



#endif