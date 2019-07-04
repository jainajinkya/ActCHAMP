#include <iostream>
#include <math.h>
#include "../include/changepoint/articulation.h"
#include "../include/changepoint/articulation_models/generic_model.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Pose.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// # define M_PI    3.141592653589793238462643383279502884L /* pi */

using namespace std;

#include <iomanip>
#define VEC(a) setprecision(5)<<fixed<<a.x()<<" "<<a.y()<<" "<<a.z()<<" "<<a.w()<<" l="<<a.length()
#define VEC2(a) "t=["<<VEC(a.getOrigin())<<"] r=[]"<<VEC(a.getRotation())<<"]"
#define PRINT(a) cout << #a <<"=" << VEC(a)<<endl;
#define PRINT2(a) cout << #a <<"=" << VEC2(a)<<endl;

geometry_msgs::Pose vectorToPose(std::vector<double>& data)
{
    geometry_msgs::Pose pos_;
    pos_.position.x = data[0];
    pos_.position.y = data[1];
    pos_.position.z = data[2];
    pos_.orientation.x = data[3];
    pos_.orientation.y = data[4];
    pos_.orientation.z = data[5];
    pos_.orientation.w = data[6];
    return pos_;
}

void helical_data_generator(double r, double pitch, int n_pts, double delta_theta, std::vector<geometry_msgs::Pose>& data)
{
    double offset = M_PI/4 ;

    for(int i=0; i<n_pts; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = r*cos(offset + i*delta_theta);
        pose.position.y = r*sin(offset + i*delta_theta);
        pose.position.z = (offset + i*delta_theta)*pitch/(2*M_PI);
        pose.orientation.w = 1.;
        std::cout << "\n " << i+1 << "-th Point:\n" << pose << std::endl;
        data.push_back(pose);
    }
}


int main(){
    // cout << "Test routine to check if models can be build and loaded." << endl;
    // articulation_models::RigidModel rigid;
    // cout << "Number of dofs of rigid: " << rigid.getDOFs() << endl;

    // articulation_models::PrismaticModel prism;
    // cout << "Number of dofs of prismatic: " << prism.getDOFs() << endl;

    // articulation_models::RotationalModel rot;
    // cout << "Number of dofs of rotational: " << rot.getDOFs() << endl;

    articulation_models::HelicalModel hel;
    // cout << "Number of dofs of helical: " << hel.getDOFs() << endl;

    // Define helix model
    tf::Vector3 rot_center(0., 0., 0.);
    tf::Quaternion rot_axis(0.,0., 0., 1.);
    tf::Quaternion rot_orientation(0.,0., 0., 1.);
    double rot_radius = 1.414;
    double hel_pitch = 10.0;

    tf::Transform trans(tf::Quaternion(0.354,-0.354,-0.146,0.854), tf::Vector3(0., 0., 0.));
    // tf::Transform trans(tf::Quaternion(0., 0., 0., 1.), tf::Vector3(0,0,0));


    std::cout << "GROUND TRUTH DATA" << std::endl;
    tf::Vector3 new_center = trans*rot_center;
    std::cout << "Rotational center: "; PRINT(new_center) ;
    tf::Quaternion new_axis = trans*rot_axis;
    std::cout << "Roation Axis: "; PRINT(new_axis) ;
    tf::Quaternion new_ori = trans*rot_orientation;
    std::cout << "Rotation Orientation: "; PRINT(new_ori);
    std::cout << "Radius: " << rot_radius << std::endl;
    std::cout << "Pitch : " << hel_pitch << std::endl;
    std::cout << "\n";

    // hel.rot_center = rot_center;
    // hel.rot_axis = rot_axis;
    // hel.rot_radius = rot_radius;
    // hel.rot_orientation = rot_orientation;
    // hel.hel_pitch = 0.1;

    geometry_msgs::Pose pose_1, pose_2, pose_3, pose_4;
    // pose_1.position.x = 1;
    // pose_1.position.y = -1;
    // pose_1.position.z = -0.0125;
    // pose_1.orientation.w = 1.;

    // pose_2.position.x = -1.;
    // pose_2.position.y = -1.;
    // pose_2.position.z = -0.0375;
    // pose_2.orientation.w = 1;
    
    // pose_3.position.x = -1.;
    // pose_3.position.y = 1.;
    // pose_3.position.z = -0.0625;
    // pose_3.orientation.w = 1;
    
    // pose_4.position.x = 1;
    // pose_4.position.y = 1.;
    // pose_4.position.z = -0.0875;
    // pose_4.orientation.w = 1;


    pose_1.position.x = 1;
    pose_1.position.y = 1;
    pose_1.position.z = 0.125;
    pose_1.orientation.x = 0.;  
    pose_1.orientation.y = 0.;  
    pose_1.orientation.z = 0.383;  
    pose_1.orientation.w = 0.924;

    pose_2.position.x = -1.;
    pose_2.position.y = 1.;
    pose_2.position.z = 0.375;
    pose_2.orientation.x = 0.;  
    pose_2.orientation.y = 0.;  
    pose_2.orientation.z = 0.924;
    pose_2.orientation.w = 0.383;  
    
    pose_3.position.x = -1.;
    pose_3.position.y = -1.;
    pose_3.position.z = 0.625;
    pose_3.orientation.x = 0.;  
    pose_3.orientation.y = 0.;  
    pose_3.orientation.z = -0.924;
    pose_3.orientation.w = 0.383;
    
    pose_4.position.x = 1;
    pose_4.position.y = -1.;
    pose_4.position.z = 0.875;
    pose_4.orientation.x = 0.;  
    pose_4.orientation.y = 0.;  
    pose_4.orientation.z = -0.383;  
    pose_4.orientation.w = 0.924;


    tf::Transform pose1 = articulation_models::poseToTransform(pose_1);
    tf::Transform pose2 = articulation_models::poseToTransform(pose_2);
    tf::Transform pose3 = articulation_models::poseToTransform(pose_3);
    tf::Transform pose4 = articulation_models::poseToTransform(pose_4);

    // Apply random Transform
    pose_1 = articulation_models::transformToPose(trans*pose1);
    pose_2 = articulation_models::transformToPose(trans*pose2);
    pose_3 = articulation_models::transformToPose(trans*pose3);
    pose_4 = articulation_models::transformToPose(trans*pose4);

    std::cout << "Helix guess Params: " << hel.guessParameters(pose_1, pose_2, pose_3, pose_4) <<std::endl;

    std::vector<geometry_msgs::Pose> data;
    helical_data_generator(rot_radius, 0.1, 4, M_PI/2, data);
}
