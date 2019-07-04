#include "../include/dynamics_models/stapler_dynamics.h"


namespace dynamics
{

/* Stapler in Revolute, Line Contact*/
class StaplerDynamicsB : public StaplerDynamics{
    public:
        StaplerDynamicsB(int n_x_rev, int n_u_rev, int n_z_rev, int n_x_free, int n_u_free, int n_z_free) : StaplerDynamics(n_x_rev, n_u_rev, n_z_rev, n_x_free, n_u_free, n_z_free){};



        // Methods
        void propagateState(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new)
        {
            tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0));
            propagateStateUseTransform(x, u, x_new, t);
        }

        void propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new, tf::Transform t)
        {            
            Eigen::VectorXd x_new_free;
            freebody->propagateState(x.head(3), u, x_new_free);
            x_new.head(3) = x_new_free;

            // Obstacles restrict motion
            if(x[1] >= y_obs)
                x_new[1] = x[1];

            // /* Theta Propagation */
            action_scale = 2.;

            tf::Transform t_center_in_ref(revolute->rot_axis, revolute->rot_center);
            tf::Transform t_ref_in_world = t;
            tf::Transform t_center_in_world = t_ref_in_world*t_center_in_ref;
            tf::Transform transformer_for_action(t_center_in_world.inverse().getRotation(), tf::Vector3(0., 0., 0.));
            
            // Scale down Inputs
            u[2] /= scale;
            x[3] /= scale;
            
            // tf::Vector3 u_applied_world(0., 0., action_scale*-u[2]);
            tf::Vector3 u_applied_world(0., 0., action_scale*u[2]);
            tf::Vector3 applied_action = transformer_for_action*(u_applied_world);
            Eigen::VectorXd u_rev = tfVectorToEigenVector(applied_action);

            // double x_new_integrate = std::asin( std::sin(x[3]-theta_flat) - (u[2]/0.19)) + theta_flat;

            // Numerical Integration            
            double x_old_integerate = x[3];
            double x_new_integrate;
            int integration_steps = 100;
            // double ang_update;

            for(int i=0; i<integration_steps; i++)
            {
                V_Configuration q_old(1);
                q_old << x_old_integerate;
                x_new_integrate = (x_old_integerate + (revolute->JacobianInverse(q_old, 1e-4)*u_rev/integration_steps)[0]);

                // ang_update = std::sin(x[3]-theta_flat) - action_scale*(u[2]/0.19);

                // THRESHOLDING ANGULAR VALUE
                // if(ang_update > 1)
                //     ang_update = 1;
                // else if(ang_update < -1)
                //     ang_update = -1;

                if(x_new_integrate > 0.)
                    x_new_integrate = 0.;
                else if(x_new_integrate < -M_PI)
                    x_new_integrate = -M_PI;

                // std::cout << "Ang update in arcsin: " << x_new_integrate << std::endl;

                // x_new_integrate = std::asin(ang_update) + theta_flat;

                // THRESHOLDING ANGULAR VALUE
                // if(x_new_integrate > M_PI/2)
                //     x_new_integrate = M_PI/2;
                // // else if(x_new_integrate < -M_PI/2)
                // //     x_new_integrate = -M_PI/2;
                // else if(x_new_integrate < 0.)
                //     x_new_integrate = 0.;

                x_old_integerate = x_new_integrate;
            }

            x_new[3] = x_new_integrate*scale;
            // std::cout << "New angle: " << x_new[3];
        }


        void getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
        {
            Eigen::VectorXd z_free;
            freebody->getObservation(x.head(3), z_free);
            z.head(3)  = z_free;

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