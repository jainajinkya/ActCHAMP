#ifndef GENERIC_KINEMATIC_PAIR_H_
#define GENERIC_KINEMATIC_PAIR_H_

#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/Pose.h"
#include <eigen3/Eigen/QR>    

#include "nonlinear_dynamics.h"
#include "kp_utils.h"

namespace dynamics{

class GenericKinematicPair: public NonlinearDynamics{
    public:
        GenericKinematicPair(int n_x, int n_u, int n_z): 
                        NonlinearDynamics(n_x, n_u, n_z){}
        ~GenericKinematicPair(){}

        double scale;  // Scaleing parameter for optimization
        Eigen::VectorXd v; // allowed_motion_vector

        // Dynamics functions
        virtual size_t getDOFs() = 0;
        virtual geometry_msgs::Pose forwardKinematics(V_Configuration q) = 0;
        virtual V_Configuration inverseKinematics(geometry_msgs::Pose pose)= 0;
        virtual void propagateState(Eigen::VectorXd, 
                Eigen::VectorXd, Eigen::VectorXd&) = 0;
        virtual void propagateStateUseTransform(Eigen::VectorXd, 
                Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0))) = 0;
        virtual void getObservation(Eigen::VectorXd, Eigen::VectorXd&) = 0;

        void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd,
            Eigen::VectorXd&, Eigen::MatrixXd&);
        void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd,
            Eigen::VectorXd&, Eigen::MatrixXd&, tf::Transform t = tf::Transform(tf::Quaternion(0., 0., 0., 1.), tf::Vector3(0.,0.,0)));
        void getObservationNoisy(Eigen::VectorXd, Eigen::VectorXd&);

        // System Matrices
        Eigen::MatrixXd A_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6);
        Eigen::MatrixXd B_mat(Eigen::VectorXd, Eigen::VectorXd, double delta=1e-6);
        Eigen::MatrixXd C_mat(Eigen::VectorXd, double delta=1e-6);
        Eigen::MatrixXd V_mat() {return 0.01*(scale*scale)*Eigen::MatrixXd::Identity(nState, nState);};
        Eigen::MatrixXd W_mat() {return 0.01*(scale*scale)*Eigen::MatrixXd::Identity(nOutput, nOutput);};
       
        // Deriavaties
        M_CartesianJacobian Jacobian(V_Configuration q, double delta = 1e-6);
        M_CartesianJacobian JacobianInverse(V_Configuration q, double delta = 1e-6);
        M_CartesianJacobian Hessian(V_Configuration q, double delta = 1e-6);

        tf::Transform getRefFrame()
        {
            std::cout << "GenericKinematicPair vala getRef" << std::endl;
            return ref_frame;
        }

        void setRefFrame(tf::Transform t)
        {
            ref_frame.setOrigin(t.getOrigin());
            ref_frame.setRotation(t.getRotation());
            std::cout << "geneirc kinematic pair vala" << std::endl;

        }
};
}

#endif // GENERIC_KINEMATIC_PAIR_H_
