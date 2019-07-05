#include "../../include/dynamics_models/generic_kinematic_pair.h"
#include <eigen3/Eigen/QR>

namespace dynamics{

Eigen::MatrixXd GenericKinematicPair::A_mat(Eigen::VectorXd x, Eigen::VectorXd u, double delta){
    Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(nState, nState);
    Eigen::VectorXd x_new(nState), x_delta(nState), x_new_delta(nState);

    propagateState(x, u, x_new);
    for(int i=0; i<nState; i++){
        x_delta = x;
        x_delta(i) += delta;
        propagateState(x_delta, u, x_new_delta);
        A_.col(i) = (x_new_delta - x_new)/delta;
    }
    return A_;
}


Eigen::MatrixXd GenericKinematicPair::B_mat(Eigen::VectorXd x, Eigen::VectorXd u, double delta){
    Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(nState, nInput);
    Eigen::VectorXd x_new(nState), u_delta(nInput), x_new_delta(nState);
    propagateState(x, u, x_new);
    for(int i=0; i<nInput; i++){
        u_delta = u;
        u_delta(i) += delta;
        propagateState(x, u_delta, x_new_delta);
        B_.col(i) = (x_new_delta - x_new)/delta;
    }
    return B_;
}


Eigen::MatrixXd GenericKinematicPair::C_mat(Eigen::VectorXd x, double delta){
    Eigen::MatrixXd C_ = Eigen::MatrixXd::Zero(nOutput, nOutput);
    Eigen::VectorXd z(nOutput), x_delta(nState), z_delta(nOutput);
    getObservation(x, z);
    for(int i=0; i<nOutput; i++){
        x_delta = x;
        x_delta(i) += delta;
        getObservation(x_delta, z_delta);
        C_.col(i) = (z_delta - z)/delta;
    }
    return C_;
}


void GenericKinematicPair::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new)
   {
        // Prediction usign EKF
        // x_new = A_mat(x,u)*x + B_mat(x,u)*u;
        propagateState(x, u, x_new);
    
        // There is no need to change the scaling of x or u, as we are calculating // A_mat and V_mat from scaled values already
        cov_new = A_mat(x,u)*cov*A_mat(x,u).transpose() + V_mat();
    }

void GenericKinematicPair::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new, tf::Transform t)
   {
        // Prediction usign EKF
        // x_new = A_mat(x,u)*x + B_mat(x,u)*u;
        propagateStateUseTransform(x, u, x_new, t);
    
        // There is no need to change the scaling of x or u, as we are calculating // A_mat and V_mat from scaled values already
        cov_new = A_mat(x,u)*cov*A_mat(x,u).transpose() + V_mat();
    }


void GenericKinematicPair::getObservationNoisy(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    getObservation(x, z);

    // Add Noise
    Eigen::VectorXd zero_mean_ = Eigen::VectorXd::Zero(nOutput);
    Eigen::MatrixXd noise_covar_ = W_mat();
    Eigen::EigenMultivariateNormal<double> normX_cholesk(zero_mean_, noise_covar_);
    Eigen::VectorXd obs_noise_ = normX_cholesk.samples(1);

    z += obs_noise_;
}


M_CartesianJacobian GenericKinematicPair::Jacobian(V_Configuration vq, double delta)
{
    M_CartesianJacobian J;
    J.resize(3, getDOFs());
    Eigen::VectorXd p = pointToEigen(forwardKinematics(vq).position);
    for(size_t i=0;i<getDOFs();i++) {
        V_Configuration q = vq;
        q(i) += scale*delta;
        J.col(i) = (pointToEigen( forwardKinematics(q).position ) - p)/delta;
    }
    // std::cout << "Jacobian: \n" << J.transpose() << std::endl;
    return J;
}

M_CartesianJacobian GenericKinematicPair::JacobianInverse(V_Configuration q, double delta)
{
    M_CartesianJacobian J(3,getDOFs()), J_inv;
    J = Jacobian(q, delta);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J); 
    J_inv = cqr.pseudoInverse();
    return J_inv;
}

M_CartesianJacobian GenericKinematicPair::Hessian(V_Configuration q, double delta)
{
    M_CartesianJacobian H;
    H.resize(3*getDOFs(),getDOFs());
//  cout <<"dofs="<<getDOFs()<<" q.size"<<vq.size()<<endl;
    for(size_t i=0;i<getDOFs();i++) {
        V_Configuration qd = q;
        q(i) += delta;
        M_CartesianJacobian H_part;

        M_CartesianJacobian J = Jacobian(q);
        M_CartesianJacobian Jd = Jacobian(qd);

//      cout << J(0,0) << " "<< J(1,0) << " "<< J(2,0) << endl;
//      cout << "H_part "<<Jd(0,0) << " "<< Jd(1,0) << " "<< Jd(2,0) << endl;

        H_part = (Jd - J)/delta;
//      cout << "H_part "<<H_part(0,0) << " "<< H_part(1,0) << " "<< H_part(2,0) << endl;
        for(size_t r=0;r<3;r++) {
            for(size_t c=0;c<getDOFs();c++) {
                H(r+3*i,c) = H_part(r,c);
            }
        }
    }
    return H;
}

}
