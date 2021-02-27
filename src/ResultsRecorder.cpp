#include <sdf_mp_integration/ResultsRecorder.h>

ResultsRecorder::ResultsRecorder(const std::string folder, const std::string file_prefix)
{
    foldername_ = folder;
    file_prefix_ = file_prefix;
    dof_ = 5;
};


void ResultsRecorder::recordTrajUpdate(const double t, const size_t num_keys, const gtsam::Values& trajectory){
    traj_updates_.push_back(std::tuple<double, size_t, gtsam::Values>(t, num_keys, trajectory));
};

const void ResultsRecorder::createSaveDir(){
        if (mkdir(foldername_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
        {
            if( errno == EEXIST ) {
                std::cout << "Folder already exists: " << strerror(errno) << std::endl;
            } else {
            // something else
                std::cout << "Cannot create folder error:" << strerror(errno) << std::endl;
                throw std::runtime_error( strerror(errno) );
            }
        }
};

const void ResultsRecorder::saveResults(){
    
    std::ofstream savefile;
    savefile.open(foldername_ + "/" + file_prefix_ + "planning_results.csv");
    
    for (size_t i = 0; i < traj_updates_.size(); i++)
    {
        gtsam::Values traj = std::get<2>(traj_updates_[i]);
        size_t num_keys = std::get<1>(traj_updates_[i]);
        double update_t = std::get<0>(traj_updates_[i]);
        
        for (size_t t_step = 0; t_step < num_keys; t_step++)
        {
            savefile << update_t << "," << num_keys << "," << t_step;

            gpmp2::Pose2Vector pos = traj.at<gpmp2::Pose2Vector>((gtsam::Symbol('x', t_step)));
            gtsam::Vector vel = traj.at<gtsam::Vector>((gtsam::Symbol('v', t_step)));

            savefile << "," << pos.pose().x();
            savefile << "," << pos.pose().y();
            savefile << "," << pos.pose().theta();

            // For each dof get position
            for (size_t j = 0; i < dof_; i++)
            {
                savefile << "," << pos.configuration()[j];
            }

            // For each dof get velocity
            for (size_t j = 0; i < dof_ + 3; i++)
            {
                savefile << "," << vel(j);
            }    

            // New line in csv
            savefile << "\n";    
        }   
    }

};