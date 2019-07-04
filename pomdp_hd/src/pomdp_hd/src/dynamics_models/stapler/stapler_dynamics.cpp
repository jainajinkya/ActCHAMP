#include "../include/dynamics_models/stapler_dynamics.h"
#include <eigen3/Eigen/QR>

namespace dynamics {

void StaplerDynamics::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new)
{
    // Prediction usign EKF
    // x_new = A_mat(x, u) * x + B_mat(x, u) * u;

    propagateState(x, u, x_new);
    cov_new = A_mat(x, u) * cov * A_mat(x, u).transpose() + V_mat();

    // std::cout << "A_mat: " << A_mat(x,u) << std::endl;
    // std::cout << "B_mat: " << B_mat(x,u) << std::endl;
    // std::cout << "x : " << x.transpose() << std::endl;
    // std::cout << "u : " << u.transpose() << std::endl;
    // std::cout << "x_new : " << x_new.transpose() << std::endl;
}


void StaplerDynamics::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new, tf::Transform t)
{
    // Prediction usign EKF
    // x_new = A_mat(x, u) * x + B_mat(x, u) * u;

    propagateStateUseTransform(x, u, x_new, t);
    cov_new = A_mat(x, u) * cov * A_mat(x, u).transpose() + V_mat();

    // std::cout << "A_mat: " << A_mat(x,u) << std::endl;
    // std::cout << "B_mat: " << B_mat(x,u) << std::endl;
    // std::cout << "x : " << x.transpose() << std::endl;
    // std::cout << "u : " << u.transpose() << std::endl;
    // std::cout << "x_new : " << x_new.transpose() << std::endl;
}


void StaplerDynamics::getObservationNoisy(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    getObservation(x, z);

    // Add Noise
    Eigen::VectorXd zero_mean_ = Eigen::VectorXd::Zero(nOutput);
    Eigen::MatrixXd noise_covar_ = W_mat();
    Eigen::EigenMultivariateNormal<double> normX_cholesk(zero_mean_, noise_covar_);
    Eigen::VectorXd obs_noise_ = normX_cholesk.samples(1);

    z += obs_noise_;
}

Eigen::MatrixXd StaplerDynamics::A_mat(Eigen::VectorXd x, Eigen::VectorXd u, double delta) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nState, nState);
    Eigen::VectorXd x_new(nState), x_delta(nState), x_new_delta(nState);

    propagateState(x, u, x_new);
    for (int i = 0; i < nState; i++) {
        x_delta = x;
        x_delta(i) += delta;
        propagateState(x_delta, u, x_new_delta);
        A.col(i) = (x_new_delta - x_new) / delta;
    }
    return A;
}

Eigen::MatrixXd StaplerDynamics::B_mat(Eigen::VectorXd x, Eigen::VectorXd u, double delta) {
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nState, nInput);
    Eigen::VectorXd x_new(nState), u_delta(nInput), x_new_delta(nState);
    propagateState(x, u, x_new);
    for (int i = 0; i < nInput; i++) {
        u_delta = u;
        u_delta(i) += delta;
        propagateState(x, u_delta, x_new_delta);
        
        // std::cout << "Update in B" << (x_new_delta - x_new).transpose()/delta << std::endl;

        B.col(i) = (x_new_delta - x_new) / delta;
    }
    // std::cout << "Estimated B Matrix: " << B << std::endl;

    return B;
}


Eigen::MatrixXd StaplerDynamics::C_mat(Eigen::VectorXd x, double delta) {
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(nState, nOutput);
    Eigen::VectorXd z(nOutput), x_delta(nState), z_delta(nOutput);
    getObservation(x, z);
    for (int i = 0; i < nOutput; i++) {
        x_delta = x;
        x_delta(i) += delta;
        getObservation(x_delta, z_delta);
        C.col(i) = (z_delta - z) / delta;
    }
    // x_delta = x;
    // x_delta(3) += delta;
    // getObservation(x_delta, z_delta);
    // C = (z_delta - z) / delta;
    return C;
}


// void StaplerDynamics::startRosNode(){
//     int argc = 1;
//     char **argv = {};
//     ros::init(argc, argv, "my_tf_listener", ros::init_options::AnonymousName);
//     ros::NodeHandle node;
// }

// tf::Transform StaplerDynamics::getRefTF(){
//     tf::StampedTransform transform;
//     tf::Transform t_;
//     try{
//         listener.lookupTransform("/linear_actuator_link", "/ar_marker_0",  
//                        ros::Time(0), transform);
//         }
//     catch (tf::TransformException ex){
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(1.0).sleep();
//     }

//     t_ = tf::Transform(tf::Quaternion(transform.getRotation()), tf::Vector3(transform.getOrigin()));
//     return t_;
// }

// Eigen::MatrixXd StaplerDynamics::V_mat() {
//     return 0.05 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
// }

// Eigen::MatrixXd StaplerDynamics::W_mat() {
//     return 0.05 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
// }


// Eigen::VectorXd StaplerDynamics::predictStaplerSecondArmCoords(V_Configuration q)
// {
//     // find coords wrt revolute frame of reference
//     geometry_msgs::Pose pose = revolute->forwardKinematics(q);

//     // Convert form revolute frame of reference to world frame
//     tf::Transform coords_in_revolute_frame = poseToTransform(pose);
//     tf::Transform coords_in_world_frame = ref_frame * coords_in_revolute_frame;

//     // return point coords
//     Eigen::VectorXd x(3);
//     x = tfVectorToEigenVector(coords_in_world_frame.getOrigin());
//     return x;
// }

// M_CartesianJacobian StaplerDynamics::Jacobian(V_Configuration vq, double delta)
// {
//     M_CartesianJacobian J;
//     J.resize(3, getDOFs());
//     Eigen::VectorXd p = pointToEigen(forwardKinematics(vq).position);
//     for(size_t i=0;i<getDOFs();i++) {
//         V_Configuration q = vq;
//         q(i) += delta;
//         J.col(i) = (pointToEigen( forwardKinematics(q).position ) - p)/delta;
//     }
//     return J;
// }

// M_CartesianJacobian StaplerDynamics::JacobianInverse(V_Configuration q, double delta)
// {
//     M_CartesianJacobian J(3,getDOFs()), J_inv;
//     J = Jacobian(q, delta);
//     Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J);
//     J_inv = cqr.pseudoInverse();
//     return J_inv;
// }

// M_CartesianJacobian StaplerDynamics::Hessian(V_Configuration q, double delta)
// {
//     M_CartesianJacobian H;
//     H.resize(3*getDOFs(),getDOFs());
// //  cout <<"dofs="<<getDOFs()<<" q.size"<<vq.size()<<endl;
//     for(size_t i=0;i<getDOFs();i++) {
//         V_Configuration qd = q;
//         q(i) += delta;
//         M_CartesianJacobian H_part;

//         M_CartesianJacobian J = Jacobian(q);
//         M_CartesianJacobian Jd = Jacobian(qd);

// //      cout << J(0,0) << " "<< J(1,0) << " "<< J(2,0) << endl;
// //      cout << "H_part "<<Jd(0,0) << " "<< Jd(1,0) << " "<< Jd(2,0) << endl;

//         H_part = (Jd - J)/delta;
// //      cout << "H_part "<<H_part(0,0) << " "<< H_part(1,0) << " "<< H_part(2,0) << endl;
//         for(size_t r=0;r<3;r++) {
//             for(size_t c=0;c<getDOFs();c++) {
//                 H(r+3*i,c) = H_part(r,c);
//             }
//         }
//     }
//     return H;
// }
}
