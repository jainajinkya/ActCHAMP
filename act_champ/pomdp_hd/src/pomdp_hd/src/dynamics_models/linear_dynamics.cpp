#include "../../include/dynamics_models/linear_dynamics.h"

namespace dynamics{

void LinearDynamics::printModelParams()
{
    std::string sep = "\n----------------------------------------\n";
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "Linear Dynamics Model Parameters: \n";
    std::cout << "Number of States: " << nState << "\n";
    std::cout << "Number of Input : " << nInput << "\n";
    std::cout << "Number of Output: " << nOutput << "\n";
    std::cout << "\nLinear Model Matrices:\n";
    std::cout << "A_mat(): \n" << A.format(CleanFmt) << "\n";
    std::cout << "B: \n" << B.format(CleanFmt) << "\n";
    std::cout << "C: \n" << C.format(CleanFmt) << "\n";
    std::cout << "V: \n" << V.format(CleanFmt) << "\n";
    std::cout << "W: \n" << W.format(CleanFmt) << "\n";
    std::cout << sep;
}

void LinearDynamics::setModelParams(int n_x, int n_u, int n_y){
        nState = n_x;
        nInput = n_u;
        nOutput = n_y;      
}

void LinearDynamics::propagateState(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new)
{
    // We don't have to do a scaling here, as A dand B are constant and the 
    // equation is linear.
    x_new = A_mat(x, u)*x + B_mat(x, u)*u;
}

void LinearDynamics::propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new, tf::Transform t )
{
    // We don't have to do a scaling here, as A dand B are constant and the 
    // equation is linear.
    x_new = A_mat(x, u)*x + B_mat(x, u)*u;
}


void LinearDynamics::getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    z = C_mat(x)*x;
}

void LinearDynamics::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new)
{
    // We don't have to do a scaling here, as A dand B are constant and the 

    propagateState(x, u, x_new);
    cov_new = A_mat(x,u)*cov*A_mat(x,u).transpose() + V_mat();
}

void LinearDynamics::propagateStateWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd u, Eigen::VectorXd& x_new, Eigen::MatrixXd& cov_new, tf::Transform t)
{
    // We don't have to do a scaling here, as A dand B are constant and the 

    propagateStateUseTransform(x, u, x_new, t);
    cov_new = A_mat(x,u)*cov*A_mat(x,u).transpose() + V_mat();
}


Eigen::VectorXd LinearDynamics::getRandomNoiseVector(Eigen::VectorXd mean)
{
    Eigen::MatrixXd noise_covar_;
    utils::nearestPD(W, noise_covar_);
    Eigen::EigenMultivariateNormal<double> normX_cholesk(mean, noise_covar_);
    return normX_cholesk.samples(1);
}


void LinearDynamics::getObservationNoisy(Eigen::VectorXd x, Eigen::VectorXd& z)
{
    Eigen::VectorXd zero_mean_ = Eigen::VectorXd::Zero(nOutput);
    Eigen::VectorXd obs_noise_ = getRandomNoiseVector(zero_mean_);
    // Generate Observation
    z = C_mat(x)*x + obs_noise_;
    z *= scale;
}

void LinearDynamics::getObservationWithCov(Eigen::VectorXd x, Eigen::MatrixXd cov, Eigen::VectorXd& z, Eigen::MatrixXd& cov_new)
{
    getObservation(x, z);
    Eigen::MatrixXd new_cov;

    new_cov = C_mat(x)*cov*C_mat(x).transpose() + W_mat();
    utils::nearestPD(new_cov, cov_new); // This ensures that cov_new is PD
}
} // end namespace
