/*
 * helical_model.cpp
 *
 * Author: Ajinkya
 */

#include "changepoint_detection/articulation_models/helical_model.h"
#include "changepoint_detection/articulation_models/utils.h"

using namespace std;
using namespace articulation_msgs;

#include <iomanip>
#define VEC(a) setprecision(5)<<fixed<<a.x()<<" "<<a.y()<<" "<<a.z()<<" "<<a.w()<<" l="<<a.length()
#define VEC2(a) "t=["<<VEC(a.getOrigin())<<"] r=[]"<<VEC(a.getRotation())<<"]"
#define PRINT(a) cout << #a <<"=" << VEC(a)<<endl;
#define PRINT2(a) cout << #a <<"=" << VEC2(a)<<endl;

namespace articulation_models {


HelicalModel::HelicalModel(){
    hel_pitch = 0.;
    complexity = 9+1;  // Rotational + 1 (pitch)
}

// -- params
void HelicalModel::readParamsFromModel() {
    RotationalModel::readParamsFromModel();
    getParam("hel_pitch", hel_pitch);
}

void HelicalModel::writeParamsToModel() {
    RotationalModel::writeParamsToModel();
    setParam("hel_pitch", hel_pitch, ParamMsg::PARAM);
}

V_Configuration HelicalModel::predictConfiguration(geometry_msgs::Pose pose) {
    V_Configuration q(1);

    // Define center of helix
    tf::Transform center(rot_axis,rot_center);

    // Transform pose from general coordinates to wrt center
    tf::Transform rel = center.inverseTimes( poseToTransform(pose) );

    // find the excess rotation b/w [-pi to pi]
    //q[0] = -atan2(rel.getOrigin().y(), rel.getOrigin().x());
    q[0] = articulation_models::remapped_atan2(rel.getOrigin().y(), rel.getOrigin().x());


    // Find n_rotations to get complete theta for helix and add it to q[0]
    double n_rotations = 0;
    if(abs(rel.getOrigin().z() > 1e-5))
        n_rotations = floor(rel.getOrigin().z()/hel_pitch);

    cout << "Before Correction, angle: " << q << endl;
    cout << "Diffrence in z val: " << rel.getOrigin().z() <<endl;   
    cout << "hel_pitch: " << hel_pitch << endl;
    cout << "Rotations encountered: " << n_rotations << endl;

    q[0] += n_rotations*2*PI; 
    cout << "Final Angle: " << q << endl;

    return q;
}

geometry_msgs::Pose HelicalModel::predictPose(V_Configuration q) {
    geometry_msgs::Pose pose;
    
    double n_rotations = q[0]/(2*PI);
     
    tf::Transform center(rot_axis, rot_center);
    tf::Transform rotZ(tf::Quaternion(tf::Vector3(0, 0, 1), -q[0]), tf::Vector3(0, 0, 0));    
    tf::Transform offsetZ(tf::Quaternion(tf::Vector3(0, 0, 1), 0), tf::Vector3(0, 0, hel_pitch*n_rotations));    
    tf::Transform r(tf::Quaternion(0,0,0,1), tf::Vector3(rot_radius, 0, 0));
    tf::Transform offset(rot_orientation, tf::Vector3(0, 0, 0));

    pose = transformToPose( center * rotZ * offsetZ * r * offset );
    
    return pose;
}


bool HelicalModel::guessParameters(geometry_msgs::Pose pose_1,geometry_msgs::Pose pose_2,geometry_msgs::Pose pose_3,geometry_msgs::Pose pose_4){
    /*
    if(model.track.pose.size() < 4)
        return false;

    size_t i;
    i = rand() % getSamples()-3;
    
    tf::Transform pose1 = poseToTransform(model.track.pose[i]);
    tf::Transform pose2 = poseToTransform(model.track.pose[i+1]);
    tf::Transform pose3 = poseToTransform(model.track.pose[i+2]);
    tf::Transform pose4 = poseToTransform(model.track.pose[i+3]);
    */

    // Determine Helical Axis // DEBUG ONLY
    tf::Transform pose1 = poseToTransform(pose_1);
    tf::Transform pose2 = poseToTransform(pose_2);
    tf::Transform pose3 = poseToTransform(pose_3);
    tf::Transform pose4 = poseToTransform(pose_4);
    
    tf::Vector3 v1 = articulation_models::angle_bisector(pose1, pose2, pose3);
    tf::Vector3 v2 = articulation_models::angle_bisector(pose2, pose3, pose4);
    v1.normalize();

    tf::Vector3 hel_axis =  v1.cross(v2);
    hel_axis.normalize();
    
    // Project points on the plane
    tf::Transform projected_pt1 = articulation_models::project_on_plane(pose2, pose1, hel_axis);
    tf::Transform projected_pt2 = articulation_models::project_on_plane(pose3, pose1, hel_axis);
    tf::Transform projected_pt3 = articulation_models::project_on_plane(pose4, pose1, hel_axis);


    // This plane is tilted at an angle from z-axis. Transform it to align with x-y plane
    tf::Vector3 plane_pos = projected_pt1.getOrigin();
    tf::Vector3 plane_v = projected_pt2.getOrigin() - projected_pt1.getOrigin();
    tf::Vector3 plane_w = projected_pt3.getOrigin() - projected_pt1.getOrigin();
    plane_v.normalize();
    plane_w.normalize();

    tf::Vector3 plane_x = plane_v;
    tf::Vector3 plane_y = plane_w - (plane_w.dot(plane_v))*plane_v;
    plane_x.normalize();
    plane_y.normalize();
    tf::Vector3 plane_z = plane_x.cross(plane_y);

    tf::Matrix3x3 plane_rot(
            plane_x.x(),plane_y.x(),plane_z.x(),
            plane_x.y(),plane_y.y(),plane_z.y(),
            plane_x.z(),plane_y.z(),plane_z.z()
            );

    tf::Transform plane(plane_rot,plane_pos);

    tf::Transform onplane_pose1 = plane.inverseTimes(projected_pt1);
    tf::Transform onplane_pose2 = plane.inverseTimes(projected_pt2);
    tf::Transform onplane_pose3 = plane.inverseTimes(projected_pt3);
    // tf::Transform onplane_pose4 = plane.inverseTimes(projected_pt4);

    tf::Vector3 p1 = (onplane_pose1.getOrigin() + onplane_pose2.getOrigin())/2;
    tf::Vector3 p21 = (onplane_pose2.getOrigin() - onplane_pose1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;

    tf::Vector3 p3 = (onplane_pose1.getOrigin() + onplane_pose3.getOrigin())/2;
    tf::Vector3 p43 = (onplane_pose3.getOrigin() - onplane_pose1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;

    tf::Vector3 p13 = p1 - p3;

    double u =  ( p43.x()*p13.y() - p43.y()*p13.x() ) /
                ( p43.y()*p21.x() - p43.x()*p21.y() );
    tf::Vector3 onplane_center = p1 + u*p21;

    tf::Transform rotcenter(plane_rot,plane * onplane_center);
    rot_center = rotcenter.getOrigin();
    rot_axis = rotcenter.getRotation();

    // Helical Pitch
    hel_pitch = 1e6; // Assuming pose1 and pose2 lie within 2pi of each other
    double dist_z = pose1.inverseTimes(pose2).getOrigin().getZ();
    V_Configuration ang_diff = predictConfiguration(pose_2) - predictConfiguration(pose_1);

    hel_pitch = abs((2*M_PI/ang_diff(0))*dist_z);

    rot_orientation = tf::Quaternion(0,0,0,1);
    // V_Configuration q = predictConfiguration( model.track.pose[i]);
    V_Configuration q = predictConfiguration(pose_1); // DEBUG ONLY
    tf::Transform pred1 = poseToTransform( predictPose(q) );
    rot_orientation = pred1.inverseTimes(pose1).getRotation();

    // DEBUG
    cout << "\nChecking if the configurations match-up with ground truth: " << endl;
    cout << "Configuration for pose 1: " << predictConfiguration(pose_1) << "\n" << endl;
    cout << "Ground Truth pose 1: \n" << pose_1 << endl;
    cout << "Recreated Configuration: \n" << predictPose(predictConfiguration(pose_1)) << "\n\n\n" << endl;

    cout << "Configuration for pose 2: " << predictConfiguration(pose_2) << "\n\n" << endl;
    cout << "Ground Truth pose 2: \n" << pose_2 << endl;
    cout << "Recreated Configuration: \n" << predictPose(predictConfiguration(pose_2)) << "\n\n\n" << endl;

    cout << "Configuration for pose 3: " << predictConfiguration(pose_3) << "\n\n" << endl;
    cout << "Ground Truth pose 3: \n" << pose_3 << endl;
    cout << "Recreated Configuration: \n" << predictPose(predictConfiguration(pose_3)) << "\n\n\n" << endl;

    cout << "Configuration for pose 4: " << predictConfiguration(pose_4) << "\n\n" << endl;
    cout << "Ground Truth pose 4: \n" << pose_4 << endl;
    cout << "Recreated Configuration: \n" << predictPose(predictConfiguration(pose_4)) << "\n\n\n" << endl;

    cout << "\nCalculated Values: " << endl;
    PRINT(rot_center);
    PRINT(rot_axis);
    PRINT(hel_axis);
    PRINT(rot_orientation);
    cout<<"rot_radius="<<rot_radius<<endl;
    cout<<"hel_pitch="<<hel_pitch<<endl;

    V_Configuration q0(1);
    q0[0] = 0.;
    cout <<  "Start point" << predictPose(q0) << endl;

    tf::Quaternion original_quat(0.35378,-0.35378,-0.14591,0.85346);
    // tf::Quaternion original_quat(0., 0., 0., 1.);

    cout << "Relative Angle " << (original_quat.inverse()*rot_axis).getAngle() << endl;


    if(!check_values(rot_center)) return false;
    if(!check_values(rot_axis)) return false;
    if(!check_values(rot_radius)) return false;
    if(!check_values(rot_orientation)) return false;

    return true;
}

void HelicalModel::updateParameters(std::vector<double> delta){
    rot_center = rot_center + tf::Vector3(delta[0],delta[1],delta[2]);
    tf::Quaternion q;
    q.setRPY(delta[3],delta[4],0.00);
    rot_axis = rot_axis * q;

    rot_radius = rot_radius + delta[5];

    tf::Quaternion q2;
    q2.setRPY(delta[6],delta[7],delta[8]);
    rot_orientation = rot_orientation * q2;

    hel_pitch = hel_pitch + delta[9];
}

bool HelicalModel::normalizeParameters(){
    // Do Something

    return true;
}
} // End of Namespace


    // // Determine centre and radius of rotation
    // tf::Vector3 p1 = (projected_pt1.getOrigin() + projected_pt2.getOrigin())/2;
    // // tf::Vector3 p21 = (projected_pt2.getOrigin() - projected_pt1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;
    // tf::Vector3 p21 = (projected_pt2.getOrigin() - projected_pt1.getOrigin()).rotate(hel_axis,M_PI/2);;


    // tf::Vector3 p3 = (projected_pt3.getOrigin() + projected_pt4.getOrigin())/2;
    // // tf::Vector3 p43 = (projected_pt3.getOrigin() - projected_pt1.getOrigin()).rotate(tf::Vector3(0,0,1),M_PI/2);;
    // tf::Vector3 p43 = (projected_pt4.getOrigin() - projected_pt3.getOrigin()).rotate(hel_axis,M_PI/2);;

    // tf::Vector3 p13 = p1 - p3;

    // double u =  ( p43.x()*p13.y() - p43.y()*p13.x() ) /
    //             ( p43.y()*p21.x() - p43.x()*p21.y() );
    // tf::Vector3 circle_center = p1 + u*p21; 


    // // Define Coordinate frame for the plane
    // tf::Vector3 plane_pos = projected_pt1.getOrigin();

    // tf::Vector3 plane_x = projected_pt2.getOrigin() - projected_pt1.getOrigin();
    // plane_x.normalize();
    // tf::Vector3 plane_z = hel_axis.normalized();
    // tf::Vector3 plane_y = plane_z.cross(plane_x);
    // plane_y.normalize();

    // tf::Matrix3x3 plane_rot(
    //         plane_x.x(),plane_y.x(),plane_z.x(),
    //         plane_x.y(),plane_y.y(),plane_z.y(),
    //         plane_x.z(),plane_y.z(),plane_z.z()
    //         );
    // tf::Transform plane(plane_rot,plane_pos);

    // tf::Transform rotcenter(plane_rot, circle_center);

    // rot_center = rotcenter.getOrigin();
    // rot_axis = rotcenter.getRotation();
    // rot_radius = (projected_pt1.getOrigin() - rot_center).length();



    // rot_orientation = tf::Quaternion(0,0,0,1);
    // // V_Configuration q = predictConfiguration( model.track.pose[i]);
    // V_Configuration q = predictConfiguration(pose_1); // DEBUG ONLY
    // tf::Transform pred1 = poseToTransform( predictPose(q) );
    // rot_orientation = pred1.inverseTimes(pose1).getRotation();
