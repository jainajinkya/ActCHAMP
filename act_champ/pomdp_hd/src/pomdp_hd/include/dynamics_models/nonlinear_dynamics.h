#ifndef NONLINEAR_DYNAMICS_H_
#define NONLINEAR_DYNAMICS_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "../eigenmvn.h"
#include "../utils.h"
#include "../dynamics.h"


namespace dynamics{

class NonlinearDynamics: public Dynamics{
    public:
        int nState = 0;
        int nInput = 0;
        int nOutput = 0;

     NonlinearDynamics(int n_x, int n_u, int n_z) : 
        nState(n_x), nInput(n_u), nOutput(n_z){}

        ~NonlinearDynamics(){}
        // params
        void printModelParams(){
            std::cout << "Number of States: " << nState << "\n";
            std::cout << "Number of Input : " << nInput << "\n";
            std::cout << "Number of Output: " << nOutput << "\n";
        };

        // Functions
        virtual void propagateState(Eigen::VectorXd, 
                Eigen::VectorXd, Eigen::VectorXd&) = 0;
        virtual void propagateStateUseTransform(Eigen::VectorXd, 
                Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t) = 0;
        virtual void getObservation(Eigen::VectorXd, Eigen::VectorXd&) = 0;
        
        virtual void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd&, Eigen::MatrixXd&) = 0;
        virtual void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd&, Eigen::MatrixXd&, tf::Transform t) = 0;
        virtual void getObservationNoisy(Eigen::VectorXd, Eigen::VectorXd&) = 0;

        // System Matrices
        virtual Eigen::MatrixXd A_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6) = 0;
        virtual Eigen::MatrixXd B_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6) = 0;
        virtual Eigen::MatrixXd C_mat(Eigen::VectorXd, double delta=1e-6) = 0;
        virtual Eigen::MatrixXd V_mat() = 0;
        virtual Eigen::MatrixXd W_mat() = 0;

        virtual tf::Transform getRefFrame() = 0;
        virtual void setRefFrame(tf::Transform t) = 0;

};
}

#endif // NONLINEAR_DYNAMICS_H_
