#include "../../include/dynamics_models/prismatic_pair.h"

namespace dynamics
{

PrismaticPair::PrismaticPair(int n_x, int n_u, int n_z) : GenericKinematicPair(n_x, n_u, n_z)
{
    rigid_position = tf::Vector3(0,0,0);
    rigid_orientation = tf::Quaternion(0,0,0,1);
    prismatic_dir = tf::Vector3(0,0,0);
}

// -- params
void PrismaticPair::printModelParams() {
    NonlinearDynamics::printModelParams();
    std::cout << "Model Name: PRISMATIC PAIR" << std::endl; 
    std::cout << "Origin : " << PRINT_VECTOR(rigid_position) << std::endl;
    std::cout << "Reference Axis : " << 
            PRINT_QUATERNION(rigid_orientation.normalized()) << std::endl;
    std::cout << "Prismatic Direction: " << PRINT_VECTOR(prismatic_dir) << std::endl;
}


geometry_msgs::Pose PrismaticPair::forwardKinematics(V_Configuration q)
{
    geometry_msgs::Pose pose;

    q /= scale;

    pose = transformToPose( tf::Transform( rigid_orientation, rigid_position + q(0) * prismatic_dir ) );

    return pose;
}


V_Configuration PrismaticPair::inverseKinematics(geometry_msgs::Pose pose)
{
    tf::Vector3 diff = (positionToVector(pose.position) - rigid_position);

    V_Configuration q(1);
    q(0) = diff.dot(prismatic_dir);

    // calculate allowed direction of motion
    v = tfVectorToEigenVector(prismatic_dir);

    return q*scale;
}


void PrismaticPair::propagateState(Eigen::VectorXd x, Eigen::VectorXd u, 
        Eigen::VectorXd& x_new)
{
    V_Configuration q(getDOFs());
    q = x/scale;
    u /= scale;

    // Euler Integratioon
    x_new = q + u*integration_step_size;

    x_new *= scale;
}
    

void PrismaticPair::propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, 
        Eigen::VectorXd& x_new, tf::Transform t)
{
    V_Configuration q(getDOFs());
    q = x/scale;
    u /= scale;

    // Euler Integratioon
    x_new = q + u*integration_step_size;

    x_new *= scale;
}
    
void PrismaticPair::getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    z = x;
}

} // end namespace
