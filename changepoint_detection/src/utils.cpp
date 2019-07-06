/*
 * utils.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: sturm
 */

#include "changepoint_detection/articulation_models/utils.h"

using namespace std;

using namespace Eigen;

namespace articulation_models {

int openChannel(articulation_msgs::TrackMsg &track,std::string name,bool autocreate) {
	// find channel
	size_t i = 0;
	for(; i < track.channels.size(); i++) {
		if(track.channels[i].name == name)
			break;
	}
	// found?
	if( i == track.channels.size() ) {
		if(!autocreate) return -1;
		// create, if not found
		sensor_msgs::ChannelFloat32 ch;
		ch.name = name;
		track.channels.push_back(ch);
	}
	// ensure that channel has right number of elements
	track.channels[i].values.resize( track.pose.size() );
	// return channel number
	return i;
}

articulation_msgs::TrackMsg flipTrack(articulation_msgs::TrackMsg input, int corner) {
	articulation_msgs::TrackMsg output = input;

	// get index of all channels
	int ch_input_w = openChannel(input,"width",false);
	int ch_input_h = openChannel(input,"height",false);
	int ch_output_w = openChannel(output,"width",false);
	int ch_output_h = openChannel(output,"height",false);

	tf::Transform in_pose,out_pose;
	double in_w=0,out_w=0;
	double in_h=0,out_h=0;

	for(size_t i=0;i<input.pose.size();i++) {
		in_pose = poseToTransform( input.pose[i] );

		if(ch_input_w>=0) in_w = input.channels[ch_input_w].values[i];
		if(ch_input_h>=0) in_h = input.channels[ch_input_h].values[i];

		switch(corner) {
		case 0:
			out_w = in_w;
			out_h = in_h;
			out_pose = in_pose * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 0 * M_PI
					/ 2), tf::Vector3(0, 0, 0));
			break;
		case 1:
		case -1:
			out_w = in_w;
			out_h = in_h;
			out_pose = in_pose * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 2 * M_PI
					/ 2), tf::Vector3(in_w, in_h, 0));
			break;
		case 2:
		case -3:
			out_w = in_h;
			out_h = in_w;
			out_pose = in_pose * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 1 * M_PI
					/ 2), tf::Vector3(in_w, 0, 0));
			break;
		case 3:
		case -2:
			out_w = in_h;
			out_h = in_w;
			out_pose = in_pose * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 3 * M_PI
					/ 2), tf::Vector3(0, in_h, 0));
			break;
		case 4:
		case -4:
			out_w = in_h;
			out_h = in_w;
			out_pose = in_pose * tf::Transform(tf::Quaternion(tf::Vector3(1, 1, 0), 2
					* M_PI / 2), tf::Vector3(0, 0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 0
					* M_PI / 2), tf::Vector3(0, 0, 0));
			break;
		case 5:
		case -5:
			out_w = in_h;
			out_h = in_w;
			out_pose =  in_pose * tf::Transform(tf::Quaternion(tf::Vector3(1, 1, 0), 2
					* M_PI / 2), tf::Vector3(0, 0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 2
					* M_PI / 2), tf::Vector3(in_h, in_w, 0));
			break;
		case 6:
		case -7:
			out_w = in_w;
			out_h = in_h;
			out_pose =  in_pose * tf::Transform(tf::Quaternion(tf::Vector3(1, 1, 0), 2
					* M_PI / 2), tf::Vector3(0, 0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 1
					* M_PI / 2), tf::Vector3(in_h, 0, 0));
			break;
		case 7:
		case -6:
			out_w = in_w;
			out_h = in_h;
			out_pose =  in_pose * tf::Transform(tf::Quaternion(tf::Vector3(1, 1, 0), 2
					* M_PI / 2), tf::Vector3(0, 0, 0)) * tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), 3
					* M_PI / 2), tf::Vector3(0, in_w, 0));
			break;
		default:
			std::cout << "WARNING: utils.cpp/flipTrack: invalid corner"<<std::endl;
			out_w = in_w;
			out_h = in_h;
			out_pose = in_pose;
		}

		output.pose[i] = transformToPose(out_pose);
		if(ch_output_w>=0) output.channels[ch_output_w].values[i] = out_w;
		if(ch_output_h>=0) output.channels[ch_output_h].values[i] = out_h;
	}

	return output;
}

Eigen::VectorXd pointToEigen(geometry_msgs::Point p) {
	Eigen::VectorXd vec(3);
	vec << p.x , p.y , p.z;
	return vec;
}

geometry_msgs::Point eigenToPoint(Eigen::VectorXd v) {
	geometry_msgs::Point p;
	p.x = v(0);
	p.y = v(1);
	p.z = v(2);
	return p;
}


//Eigen::VectorXd vectorToEigen(V_Configuration q) {
//	Eigen::VectorXd vec(q.size());
//	for(size_t i=0;i<q.size();i++)
//		vec[i] = q[i];
//	return vec;
//}
//
//Eigen::MatrixXd matrixToEigen(M_CartesianJacobian J) {
//	Eigen::MatrixXd m(J.size(),3);
//
//	for(size_t i=0;i<J.size();i++) {
//		m(i,0) = J[i].x;
//		m(i,1) = J[i].y;
//		m(i,2) = J[i].z;
//	}
//	return m;
//}

void setParamIfNotDefined(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name, double value, uint8_t type) {
	for (size_t i = 0; i < vec.size(); i++)
		if (vec[i].name == name)
			return;
	articulation_msgs::ParamMsg param;
	param.name = name;
	param.value = value;
	param.type = type;
	vec.push_back(param);
}

void setParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name, double value, uint8_t type) {
	for (size_t i = 0; i < vec.size(); i++) {
		if (vec[i].name == name) {
			vec[i].value = value;
			vec[i].type = type;
			return;
		}
	}
	articulation_msgs::ParamMsg param;
	param.name = name;
	param.value = value;
	param.type = type;
	vec.push_back(param);
}

double getParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name) {
	for (size_t i = 0; i < vec.size(); i++) {
		if (vec[i].name == name) {
			return vec[i].value;
		}
	}
	return 0;
}

bool hasParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name) {
	for (size_t i = 0; i < vec.size(); i++) {
		if (vec[i].name == name) {
			return true;
		}
	}
	return false;
}


bool check_values(const tf::Vector3 &vec) {
	return(check_values(vec.x()) && check_values(vec.y()) && check_values(vec.z()) );
}
bool check_values(const tf::Quaternion &vec) {
	return(check_values(vec.x()) && check_values(vec.y()) && check_values(vec.z()) && check_values(vec.w()));
}
bool check_values(double v) {
	return(!isnan(v) && !isinf(v));
}
bool check_values(float v) {
	return(!isnan(v) && !isinf(v));
}
bool parameter_too_large(double v, double ref) {
	return(v>ref);
}

double remapped_atan2(double y, double x){
    double val = atan2(y, x);
    std::cout << "Value from atan2: " << val << std::endl;
    if(val >= 0)
        return val;
    else
       return 2*PI + val; 
}

tf::Vector3 angle_bisector(tf::Transform pt1, tf::Transform pt2, tf::Transform pt3){
    tf::Vector3 a = pt2.getOrigin() - pt1.getOrigin();
    tf::Vector3 b = pt3.getOrigin() - pt2.getOrigin();
    tf::Vector3 c = b.length()*a + a.length()*b;
    c.normalize();
    return c;
}

tf::Transform project_on_plane(tf::Transform pose, tf::Transform orig, tf::Vector3 normal){
    normal.normalize();
    tf::Vector3 v = pose.getOrigin() - orig.getOrigin();
    tf::Vector3 projected_pt = pose.getOrigin() - (v.dot(normal))*normal;

    tf::Vector3 plane_x = (projected_pt - orig.getOrigin()).normalized();
    tf::Vector3 plane_z = normal.normalized();
    tf::Vector3 plane_y = plane_z.cross(plane_x);

    tf::Matrix3x3 plane_rot(
        plane_x.x(),plane_y.x(),plane_z.x(),
        plane_x.y(),plane_y.y(),plane_z.y(),
        plane_x.z(),plane_y.z(),plane_z.z()
        );

    tf::Transform projection(plane_rot, projected_pt);
    return projection;
}

tf::Transform perpendicular_bisector(tf::Transform pt1, tf::Transform pt2, tf::Vector3 helper_axis){
    tf::Vector3 mid_pt = (pt1.getOrigin() + pt2.getOrigin())/2;
    tf::Vector3 line_dir = pt2.getOrigin() - pt1.getOrigin();
    line_dir.normalize();
    helper_axis.normalize();
    tf::Vector3 pb_dir = line_dir.cross(helper_axis).normalize();
    tf::Transform pb(tf::Quaternion(pb_dir, 1.), mid_pt);
    return pb;
}

} // end namespace articulation_models
