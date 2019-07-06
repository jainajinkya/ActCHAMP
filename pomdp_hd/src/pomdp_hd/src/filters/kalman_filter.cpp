#include "../../include/filters/kalman_filter.h"

namespace filters
{
KalmanFilter::KalmanFilter()
{
    // Define Dynamics
    ProblemDefinition();
}

void KalmanFilter::prediction(Eigen::VectorXd x_hat, 
        Eigen::MatrixXd P_hat, Eigen::VectorXd u, 
        Eigen::VectorXd& x_bar, Eigen::MatrixXd& P_bar)
{
    int activeIdx;
    utils::activeModel(x_hat, activeIdx, nState, nModel, set_of_gcs);
    dynamics[activeIdx]->propagateStateWithCov(x_hat, P_hat, u, x_bar, P_bar, s_to_LA);
}

void KalmanFilter::update(Eigen::VectorXd x_bar, 
        Eigen::MatrixXd P_bar, Eigen::VectorXd z, 
        Eigen::VectorXd& x_new, Eigen::MatrixXd& P_new)
{
    int activeIdx;
    Eigen::MatrixXd C(nOutput, nOutput), W(nOutput, nOutput), K(nState, nOutput);
    Eigen::VectorXd z_model(nOutput);
    
    Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(nOutput, nOutput);
    utils::activeModel(x_bar, activeIdx, nState, nModel, set_of_gcs);
    dynamics[activeIdx]->getObservation(x_bar, z_model);
    C = dynamics[activeIdx]->C_mat(x_bar);
    W = dynamics[activeIdx]->W_mat();

    Eigen::MatrixXd cov_residue(nOutput, nOutput);
    cov_residue = C*P_bar*C.transpose() + W;
    utils::nearestPD(cov_residue, cov_residue); // This ensures that cov_new is PD

    K = P_bar*C.transpose()*cov_residue.inverse();
    /*
    std::cout << "K :" << K << std::endl;
    std::cout << "x_bar :" << x_bar.transpose() << std::endl;
    std::cout << "P_bar :\n" << P_bar << std::endl;
    std::cout << "Innovations" << std::endl;
    std::cout << "in x :" << (K*(z - z_model)).transpose() << std::endl;
    //std::cout << "in P :\n" << P_bar << std::endl;
    */
    x_new = x_bar + K*(z - z_model);
    P_new = (I_ - K*C)*P_bar*(I_ - K*C).transpose() + K*W*K.transpose();
}

} // end namespace

