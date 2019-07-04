#include "../include/dynamics_models/stapler_dynamics.h"

namespace dynamics
{

/* Stapler in Rigid */
class StaplerDynamicsA : public StaplerDynamics{
    public:
        StaplerDynamicsA(int n_x_rev, int n_u_rev, int n_z_rev, int n_x_free, int n_u_free, int n_z_free) : StaplerDynamics(n_x_rev, n_u_rev, n_z_rev, n_x_free, n_u_free, n_z_free){};


        // Methods
        void propagateState(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new)
        {
            tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0));
            propagateStateUseTransform(x, u, x_new, t);
        }

        
        void propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new, tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0)))
        {
            Eigen::VectorXd x_new_free;
            freebody->propagateState(x.head(3), u, x_new_free);
            x_new.head(3) = x_new_free;

            // Obstacles restrict motion
            if(x[1] >= y_obs && (x[2] <= z_obs || x[2] >= z_obs_top))
                x_new[1] = x[1];
            
            x_new[3] = theta_fixed*scale;

            // std::cout << "\n\nx cur: " << x.transpose() << "\nu : " << u.transpose() << "\n x new: " << x_new.transpose() << std::endl;
        }

        void getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
        {
            Eigen::VectorXd z_free;
            freebody->getObservation(x.head(3), z_free);
            z.head(3) = z_free;

            z[3] = x[3];

        }   

        Eigen::MatrixXd V_mat() {
            Eigen::MatrixXd V_ = 0.05 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
            V_(3,3) = 1e-6;
            return V_;
        }

        Eigen::MatrixXd W_mat() {
            Eigen::MatrixXd W_ = 0.5 * 0.05 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
            W_(3,3) = 1e-6;
            return W_;
        }

};
}