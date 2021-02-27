#include <sdf_mp_integration/utils/Visualiser.h>

template <class ModelType>
Visualiser<ModelType>::Visualiser(ros::NodeHandle n){
    n_ = n;
    marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
};

template <class ModelType>
void Visualiser<ModelType>::setArm(ModelType& arm_model){
    arm_model_ = arm_model;
    num_spheres_ = arm_model_.nr_body_spheres();

    for (size_t i = 0; i < num_spheres_; i++)
    {
        robot_spheres_radii_.push_back(arm_model_.sphere_radius(i));
    }

};

template <>
const void Visualiser<gpmp2::Pose2MobileVetLinArmModel>::visualiseRobotAxes(const gpmp2::Pose2Vector &conf, const int id) {
// const void Visualiser<ModelType>::visualiseRobot(const gtsam::Vector &conf, const int id) {

    std::vector<gtsam::Point3> sph_centers;
    arm_model_.sphereCenters(conf, sph_centers);
    
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(3*num_spheres_/4);

    std::cout << "Publishing markers" << std::endl;

    for (size_t i = 0; i < num_spheres_/4 ; i++)
    {
        visualization_msgs::Marker marker;
        std::cout << "Joint" << i << std::endl;

        gtsam::Point3 dx = sph_centers[4*i+1] - sph_centers[4*i];
        gtsam::Point3 dy = sph_centers[4*i+2] - sph_centers[4*i];
        gtsam::Point3 dz = sph_centers[4*i+3] - sph_centers[4*i];

        Eigen::Matrix3d pose;
        pose << dx[0], dy[0], dz[0],
                dx[1], dy[1], dz[1], 
                dx[2], dy[2], dz[2];

        pose.col(0).normalize();
        pose.col(1).normalize();
        pose.col(2).normalize();

        Eigen::Isometry3d xpose(pose);
        Eigen::Isometry3d ypose = xpose * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
        Eigen::Isometry3d zpose = xpose * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());

        Eigen::Quaterniond xq(xpose.rotation());

        Eigen::Quaterniond yq(ypose.rotation());
        Eigen::Quaterniond zq(zpose.rotation());

        marker.pose.position.x = sph_centers[4*i][0];
        marker.pose.position.y = sph_centers[4*i][1];
        marker.pose.position.z = sph_centers[4*i][2];

        // Create a marker  
        marker.header.frame_id = "odom";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.ns = "occupancy";
        marker.lifetime = ros::Duration(2);
        marker.color.a = 1.0;
        

        // x axis
        marker.id = id+4*i;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;        
        marker.pose.orientation.w = xq.w();
        marker.pose.orientation.x = xq.x();
        marker.pose.orientation.y = xq.y();
        marker.pose.orientation.z = xq.z();
        // marker_pub_.publish(marker);
        marker_array.markers[3*i] = marker;

        
        // y axis
        marker.id = id+4*i+1;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;           
        marker.pose.orientation.w = yq.w();
        marker.pose.orientation.x = yq.x();
        marker.pose.orientation.y = yq.y();
        marker.pose.orientation.z = yq.z();
        // marker_pub_.publish(marker);
        marker_array.markers[(3*i) + 1] = marker;


        // z axis
        marker.id = id+4*i+2;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;     
        marker.pose.orientation.w = zq.w();
        marker.pose.orientation.x = zq.x();
        marker.pose.orientation.y = zq.y();
        marker.pose.orientation.z = zq.z();      
        // marker_pub_.publish(marker);
        marker_array.markers[(3*i) + 2] = marker;
        // std::cout << "Markers published" << std::endl;

    }
    marker_pub_.publish(marker_array);

}

template <>
const void Visualiser<gpmp2::Pose2MobileVetLinArmModel>::visualiseRobot(const gpmp2::Pose2Vector &conf, const int id) {
// const void Visualiser<ModelType>::visualiseRobot(const gtsam::Vector &conf, const int id) {
  
    std::vector<gtsam::Point3> sph_centers;
    arm_model_.sphereCenters(conf, sph_centers);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(num_spheres_);

    for (size_t i = 0; i < num_spheres_; i++)
    {
        visualization_msgs::Marker marker;

        // Create a marker  
        marker.header.frame_id = "odom";
        marker.id = id+i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = robot_spheres_radii_[i];
        marker.scale.y = robot_spheres_radii_[i];
        marker.scale.z = robot_spheres_radii_[i];
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;        
        marker.pose.position.x = sph_centers[i](0);
        marker.pose.position.y = sph_centers[i](1);
        marker.pose.position.z = sph_centers[i](2);
        marker.ns = "occupancy";
        marker.lifetime = ros::Duration(1);

        // Points are green
        marker.color.r = 1.0f;
        marker.color.a = 1.0;
        // std::cout << "Sent robot sphere marker: " << i << std::endl;
        marker_array.markers[i] = marker;
    }
    marker_pub_.publish(marker_array);

}

template <>
const void Visualiser<gpmp2::ArmModel>::visualiseRobot(const gtsam::Vector &conf, const int id) {
// const void Visualiser<ModelType>::visualiseRobot(const gtsam::Vector &conf, const int id) {
  
    std::vector<gtsam::Point3> sph_centers;
    arm_model_.sphereCenters(conf, sph_centers);

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(num_spheres_);

    for (size_t i = 0; i < num_spheres_; i++)
    {
        visualization_msgs::Marker marker;

        // Create a marker  
        marker.header.frame_id = "odom";
        marker.id = id+i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = robot_spheres_radii_[i];
        marker.scale.y = robot_spheres_radii_[i];
        marker.scale.z = robot_spheres_radii_[i];
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;        
        marker.pose.position.x = sph_centers[i](0);
        marker.pose.position.y = sph_centers[i](1);
        marker.pose.position.z = sph_centers[i](2);
        marker.ns = "occupancy";
        marker.lifetime = ros::Duration(1);


        // Points are green
        marker.color.r = 1.0f;
        marker.color.a = 1.0;
        // std::cout << "Sent robot sphere marker" << std::endl;
        marker_array.markers[i] = marker;
    }
    marker_pub_.publish(marker_array);

}

template class Visualiser<gpmp2::Pose2MobileVetLinArmModel>;
template class  Visualiser<gpmp2::ArmModel>;
