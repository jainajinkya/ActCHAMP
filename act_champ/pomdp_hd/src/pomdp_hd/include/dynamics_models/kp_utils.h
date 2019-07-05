/*
 * utils.h
 *
 *  Created on: Oct 21, 2009
 *      Author: Jurgen Sturm, Ajinkya Jain
 */

#ifndef KP_UTILS_H_
#define KP_UTILS_H_

#include "tf/LinearMath/Transform.h"
#include <Eigen/Core>

namespace dynamics{

#ifndef SQR
#define SQR(a) ((a)*(a))
#endif

#ifndef MIN
#define MIN(a,b) ((a<=b)?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a>=b)?(a):(b))
#endif

#define PI 3.14159265

#define PRINT_TRANSFORM(tf) \
	"[ "<<tf.getOrigin().x() <<"; "<<tf.getOrigin().y() <<"; "<<tf.getOrigin().z() <<"]" << \
	"( "<<tf.getRotation().x() <<"; "<<tf.getRotation().y() <<"; "<<tf.getRotation().z()<<"; "<<tf.getRotation().w() <<") "

#define PRINT_VECTOR(vec) \
	"[ "<<vec.getX() <<"; "<<vec.getY() <<"; "<<vec.getZ() <<"]"

#define PRINT_QUATERNION(quat) \
    "( "<<quat.getAxis().getX() <<"; "<< quat.getAxis().getY() <<"; "<<quat.getAxis().getZ() <<") "

typedef Eigen::MatrixXd M_CartesianJacobian;
typedef Eigen::VectorXd V_Configuration;

inline tf::Quaternion orientationToQuaternion(geometry_msgs::Quaternion orientation) {
	return tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w);
}

inline tf::Vector3 positionToVector(geometry_msgs::Point position) {
	return tf::Vector3(position.x,position.y,position.z);
}

inline tf::Transform poseToTransform(geometry_msgs::Pose pose) {
	return(tf::Transform(
			orientationToQuaternion(pose.orientation),
			positionToVector(pose.position)
		));
}

inline geometry_msgs::Quaternion quaternionToOrientation(tf::Quaternion quat) {
	geometry_msgs::Quaternion orientation;
	orientation.x = quat.x();
	orientation.y = quat.y();
	orientation.z = quat.z();
	orientation.w = quat.w();
	return orientation;
}

inline geometry_msgs::Point vectorToPosition(tf::Vector3 point) {
	geometry_msgs::Point position;
	position.x = point.x();
	position.y = point.y();
	position.z = point.z();
	return position;
}

inline geometry_msgs::Pose transformToPose(tf::Transform transform) {
	geometry_msgs::Pose pose;
	pose.orientation = quaternionToOrientation( transform.getRotation() );
	pose.position = vectorToPosition( transform.getOrigin() );
	return pose;
}


inline Eigen::VectorXd pointToEigen(geometry_msgs::Point p) {
	Eigen::VectorXd vec(3);
	vec << p.x , p.y , p.z;
	return vec;
}

inline geometry_msgs::Point eigenToPoint(Eigen::VectorXd v) {
	geometry_msgs::Point p;
	p.x = v(0);
	p.y = v(1);
	p.z = v(2);
	return p;
}

// Eigen::VectorXd vectorToEigen(V_Configuration q);
// Eigen::MatrixXd matrixToEigen(M_CartesianJacobian J);

inline geometry_msgs::Pose vectorToPose(Eigen::VectorXd x){
    geometry_msgs::Pose pose;
    pose.position.x = x(0);
    pose.position.y = x(1);
    pose.position.z = x(2);
    pose.orientation.x = x(3);
    pose.orientation.y = x(4);
    pose.orientation.z = x(5);
    pose.orientation.w = x(6);
    return pose;
}

inline Eigen::VectorXd poseToVector(geometry_msgs::Pose pose){
    Eigen::VectorXd x(7);
    x << pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w;
    return x;
}

inline Eigen::VectorXd tfVectorToEigenVector(tf::Vector3 vec){
    Eigen::VectorXd arr(3);
    arr << vec.getX(), vec.getY(), vec.getZ();
    return arr;
}

inline tf::Vector3 eigenVectorToTfVector(Eigen::VectorXd arr){
    tf::Vector3 vec(arr[0], arr[1], arr[2]);
    return vec;
}
}
#endif /* KP_UTILS_H_ */
