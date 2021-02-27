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