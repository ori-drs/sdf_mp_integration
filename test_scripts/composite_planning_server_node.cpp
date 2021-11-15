#include <ros/ros.h>
#include <sdf_mp_integration/PlanningServer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "planning_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    sdf_mp_integration::PlanningServer<gpu_voxels_ros::LiveCompositeSDF*>* planning_server_ptr; 
    // sdf_mp_integration::PlanningServer<gpu_voxels_ros::GPUVoxelsHSRServer*>* planning_server_ptr; 
    
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::cout << "Allocated the Planning Server." << std::endl;
    planning_server_ptr = new sdf_mp_integration::PlanningServer<gpu_voxels_ros::LiveCompositeSDF*>(nh);
    // planning_server_ptr = new sdf_mp_integration::PlanningServer<gpu_voxels_ros::GPUVoxelsHSRServer*>(nh);

    ros::waitForShutdown();

    return 0;
}
