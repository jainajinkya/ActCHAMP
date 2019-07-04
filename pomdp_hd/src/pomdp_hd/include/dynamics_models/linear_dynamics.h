#ifndef LINEAR_DYNAMICS_H_
#define LINEAR_DYNAMICS_H_

#include "../dynamics.h"

namespace dynamics{

class LinearDynamics: public Dynamics{
public:
    // System Matrices
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd V;
    Eigen::MatrixXd W;
    
    LinearDynamics(int nx, int nu, int nz){
        nState = nx;
        nInput = nu;
        nOutput = nz;

        A = Eigen::MatrixXd::Identity(nState, nState);
        B = Eigen::MatrixXd::Identity(nState, nInput);
        C = Eigen::MatrixXd::Identity(nOutput, nOutput);
        V = Eigen::MatrixXd::Identity(nState, nState);
        W = Eigen::MatrixXd::Identity(nOutput, nOutput);
    }

   ~LinearDynamics(){;}
   
    // params
    double scale = 1; // Planning Scale

    // Methods
    void printModelParams();
    void setModelParams(int n_x, int n_u, int n_y);

    // Beleif Propagation Functions
    void propagateState(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&);
    void propagateStateUseTransform(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&, tf::Transform);

    void getObservation(Eigen::VectorXd, Eigen::VectorXd&);

    void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, 
                        Eigen::VectorXd, Eigen::VectorXd&, 
                        Eigen::MatrixXd&);
    void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, 
                        Eigen::VectorXd, Eigen::VectorXd&, 
                        Eigen::MatrixXd&, tf::Transform);

    void getObservationNoisy(Eigen::VectorXd, Eigen::VectorXd&);

    // System Matrices
    Eigen::MatrixXd A_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6) {return A;}
    Eigen::MatrixXd B_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6) {return B;}
    Eigen::MatrixXd C_mat(Eigen::VectorXd, double delta=1e-6) {return C;}
    Eigen::MatrixXd V_mat() {return V;}
    Eigen::MatrixXd W_mat() {return W;}

    // Other Functions
    Eigen::VectorXd getRandomNoiseVector(Eigen::VectorXd);
    void getObservationWithCov(Eigen::VectorXd, 
            Eigen::MatrixXd, Eigen::VectorXd&, Eigen::MatrixXd&);

    // Redundant Functions
    geometry_msgs::Pose forwardKinematics(Eigen::VectorXd){
        geometry_msgs::Pose pose_;
        return pose_;
    }

    Eigen::VectorXd inverseKinematics(geometry_msgs::Pose){
        Eigen::VectorXd q(1);
        q << 0.;
        return q;
    };

    tf::Transform getRefFrame()
    {
        std::cout << "LinearDynamics vala getRef" << std::endl;
        return ref_frame;
    }

    void setRefFrame(tf::Transform t)
    {
        ref_frame.setOrigin(t.getOrigin());
        ref_frame.setRotation(t.getRotation());
        std::cout << "linear dynamics vala" << std::endl;
    }
};
}
#endif // LINEAR_DYNAMICS_H_
