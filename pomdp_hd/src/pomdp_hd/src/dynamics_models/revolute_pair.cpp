#include "../../include/dynamics_models/revolute_pair.h"
#include <stdexcept>

namespace dynamics
{

RevolutePair::RevolutePair(int n_x, int n_u, int n_z) : GenericKinematicPair(n_x, n_u, n_z)
{
	rot_center = tf::Vector3 (0,0,0);
	rot_axis = tf::Quaternion (0,0,0,1);
	rot_radius = 1.;
	rot_orientation = tf::Quaternion (0,0,0,1);  
}

// -- params
void RevolutePair::printModelParams() {
    NonlinearDynamics::printModelParams();
    std::cout << "Model Name: REVOLUTE PAIR" << std::endl; 
    std::cout << "Center : " << PRINT_VECTOR(rot_center) << std::endl;
    std::cout << "Axis : " << PRINT_QUATERNION(rot_axis.normalized()) << std::endl;
    std::cout << "Radius : " << rot_radius << std::endl;
    std::cout << "Radial Vector Orientation: " \
        << PRINT_QUATERNION(rot_orientation.normalized()) << std::endl;
}


geometry_msgs::Pose RevolutePair::forwardKinematics(V_Configuration q)
{
	geometry_msgs::Pose pose;

    q /= scale;

    tf::Transform center(rot_axis,rot_center);
    tf::Transform rotZ(tf::Quaternion(tf::Vector3(0, 0, 1), -q[0]), tf::Vector3(0, 0, 0));
    tf::Transform r(tf::Quaternion(0,0,0,1), tf::Vector3(rot_radius, 0, 0));
    tf::Transform offset(rot_orientation, tf::Vector3(0, 0, 0));

	pose = transformToPose( center * rotZ * r * offset );

	return pose;
}


V_Configuration RevolutePair::inverseKinematics(geometry_msgs::Pose pose)
{
	V_Configuration q(1);
	tf::Transform center(rot_axis,rot_center);
	tf::Transform rel = center.inverseTimes( poseToTransform(pose) );
    q(0) = -atan2(rel.getOrigin().y(), rel.getOrigin().x());

    // calculate allowed direction of motion
    tf::Vector3 axis(0,0,1);
    tf::Vector3 r = rel.getOrigin();
    v = tfVectorToEigenVector(center*axis.cross(r));
    // std::cout << "\nAllowed Motion Vector: " << v.transpose() << std::endl;

    return q*scale;
}


void RevolutePair::propagateState(Eigen::VectorXd x, Eigen::VectorXd u, 
        Eigen::VectorXd& x_new)
{
    V_Configuration q(getDOFs());
    q = x/scale;
    u /= scale;

    // Euler Integratioon
    x_new = q + u*integration_step_size;

    x_new *= scale;

    // if(u.size() > getDOFs()) {
    //     // q_new = inverseKinematics(vectorToPose(x));
    //     // std::cout << "Configuration from IK: " << q_new  << std::endl; 

    //     M_CartesianJacobian J_inv;    
    //     J_inv = JacobianInverse(q_new);
    //     // std::cout << "Jacobian Inverse: " << J_inv << std::endl;

    //     x_new = poseToVector(forwardKinematics(q_new + J_inv*(u.dot(v)*v)));
    //     // std::cout << "Check using FK: \n" << forwardKinematics(q_new) << std::endl; 
    //     // std::cout << "projection : " << u.dot(v)*v << std::endl;
    //     // std::cout << "Second term : " << J_inv*(u.dot(v)*v) << std::endl;
    // }
}

void RevolutePair::propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, 
        Eigen::VectorXd& x_new, tf::Transform t)
{
    V_Configuration q(getDOFs());
    q = x/scale;
    u /= scale;

    // Euler Integratioon
    x_new = q + u*integration_step_size;

    x_new *= scale;

    // if(u.size() > getDOFs()) {
    //     // q_new = inverseKinematics(vectorToPose(x));
    //     // std::cout << "Configuration from IK: " << q_new  << std::endl; 

    //     M_CartesianJacobian J_inv;    
    //     J_inv = JacobianInverse(q_new);
    //     // std::cout << "Jacobian Inverse: " << J_inv << std::endl;

    //     x_new = poseToVector(forwardKinematics(q_new + J_inv*(u.dot(v)*v)));
    //     // std::cout << "Check using FK: \n" << forwardKinematics(q_new) << std::endl; 
    //     // std::cout << "projection : " << u.dot(v)*v << std::endl;
    //     // std::cout << "Second term : " << J_inv*(u.dot(v)*v) << std::endl;
    // }
}
    
void RevolutePair::getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    z = x;
}

} // end namespace
