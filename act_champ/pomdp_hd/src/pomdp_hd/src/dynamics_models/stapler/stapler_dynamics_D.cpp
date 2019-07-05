#include "../include/dynamics_models/stapler_dynamics.h"
#include "tf/transform_datatypes.h"

namespace dynamics
{

/* Stapler in Revolute mode, Surface Contact */
class StaplerDynamicsD : public StaplerDynamics{
    public:
        StaplerDynamicsD(int n_x_rev, int n_u_rev, int n_z_rev, int n_x_free, int n_u_free, int n_z_free) : StaplerDynamics(n_x_rev, n_u_rev, n_z_rev, n_x_free, n_u_free, n_z_free){};


        void propagateState(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new)
        {
            tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0));
            propagateStateUseTransform(x, u, x_new, t);
        }
        

        void propagateStateUseTransform(Eigen::VectorXd x, Eigen::VectorXd u, Eigen::VectorXd& x_new, tf::Transform t)
        {
            Eigen::VectorXd x_new_free;
            Eigen::VectorXd u_eff(3);
            u_eff << u[0], u[1], 0.;

            freebody->propagateState(x.head(3), u_eff, x_new_free);
            x_new.head(3) = x_new_free;

            // Obstacles restrict motion
            if(x[1] >= y_obs)
                x_new[1] = x[1];


            // /* Theta Propagation */
            // action_scale = 2*1.585;
            action_scale = 1.916;
            
            tf::Transform t_center_in_ref(revolute->rot_axis, revolute->rot_center);
            tf::Transform t_ref_in_world = t;
            tf::Transform t_center_in_world = t_ref_in_world*t_center_in_ref;
            tf::Transform transformer_for_action(t_center_in_world.inverse().getRotation(), tf::Vector3(0., 0., 0.));

            // Scale Inputs down
            u[2] /= scale;
            x[3] /= scale;
           
            // tf::Vector3 u_applied_world(0., 0., action_scale*-u[2]);
            tf::Vector3 u_applied_world(0., 0., action_scale*u[2]);
            tf::Vector3 applied_action = transformer_for_action*(u_applied_world);
            Eigen::VectorXd u_rev = tfVectorToEigenVector(applied_action);
            
            double x_old_integerate = x[3];
            double x_new_integrate;
            int integration_steps = 100;

            for(int i=0; i<integration_steps; i++)
            {
                V_Configuration q_old(1);
                q_old << x_old_integerate;
                x_new_integrate = (x_old_integerate + (revolute->JacobianInverse(q_old, 1e-4)*u_rev/integration_steps)[0]);

                // THRESHOLDING ANGULAR VALUE
                // if(x_new_integrate > M_PI/2)
                //     x_new_integrate = M_PI/2;
                // // else if(x_new_integrate < -M_PI/2)
                // //     x_new_integrate = -M_PI/2;
                // else if(x_new_integrate < 0.)
                //     x_new_integrate = 0.;

                if(x_new_integrate > 0.)
                    x_new_integrate = 0.;
                else if(x_new_integrate < -M_PI)
                    x_new_integrate = -M_PI;
                
                x_old_integerate = x_new_integrate;
            }

            x_new[3] = x_new_integrate*scale;


            // std::cout << "new_angle : " << x_new[3] << std::endl;

            //*/ Debugging */
            // std::cout << "t_center_in_ref: " << PRINT_TRANSFORM(t_center_in_ref) << std::endl;
            // std::cout << "t_ref_in_world: " << PRINT_TRANSFORM(t_ref_in_world) << std::endl;

            // std::cout << "t_center_in_world: " << PRINT_TRANSFORM(t_center_in_world) << std::endl;
            
            // std::cout << "t_center_in_world: " << PRINT_TRANSFORM(t_center_in_world) << std::endl;
            // std::cout << "transformer_for_action: " << PRINT_TRANSFORM(transformer_for_action) << std::endl;

            // dynamics::V_Configuration q(1);
            // q << x_new[3];
            // geometry_msgs::Pose pose_pred_local = revolute->forwardKinematics(q);
            // geometry_msgs::Transform pose_pref_local_tf;
            // tf::Transform pose_pred_tf, pose_pred_tf_world;

            // tf2::convert(pose_pred_local, pose_pref_local_tf);
            // tf::transformMsgToTF(pose_pref_local_tf, pose_pred_tf);
            // pose_pred_tf_world = t*pose_pred_tf;

            // std::cout << "\n Delta theta: " << x_new[3] - x[3] << std::endl;
            // std::cout << "\n Applied action: " << u_rev.transpose() << std::endl;


            // std::cout << "Z new in world,\tFree Body: " << x_new[2] << "\tRevolute: " << pose_pred_tf_world.getOrigin().getZ() << std::endl;

            // std::cout << "Delta z in world,\tPlanned: " << u[2] << "\tActual: " << pose_pred_tf_world.getOrigin().getZ() - x[2] << std::endl;




            /* DEPRECATED*/
            // geometry_msgs::Pose pose_old;
            // pose_old.position.x = x[0];
            // pose_old.position.y = x[1];
            // pose_old.position.z = x[2];
            // // pose_old.orientation.w = 1.;
            // tf::quaternionTFToMsg(ref_frame.getRotation(), pose_old.orientation);

            // V_Configuration q_old = revolute->inverseKinematics(pose_old);
            // std::cout << "Old Angle: " << q_old << std::endl;

            // double zero_val = -65.3027;
            // geometry_msgs::Pose pose;
            // pose.position.x = x_new[0];
            // pose.position.y = x_new[1];
            // pose.position.z = x_new[2];
            // pose.orientation.w = 1.;
            // // tf::quaternionTFToMsg(ref_frame.getRotation(), pose.orientation);

            // V_Configuration q = revolute->inverseKinematics(pose);
            // // x_new[3] = M_PI*scale + (zero_val - q[0]); 
            // x_new[3] = q[0];

            // std::cout << "Angle before correction: " << q[0] << std::endl;
            // std::cout << "\nCorrection: " << (zero_val - q[0]) << std::endl;
            // std::cout << "New Angle: " << x_new[3] << std::endl;
            // std::cout << "x_new: " << x_new.transpose() << std::endl;

            // // Method 2
            // // Control Input to the revolute part will be in Cartesian Space, 
            // // but it must be defined in the frame of reference for the revolute part


            // u[2] *=  -1.; // Only the input vector in z direction needs to reveresed
            // tf::Transform center(revolute->rot_axis,revolute->rot_center);

            // tf::Transform t1(revolute->rot_axis, tf::Vector3(0., 0.,0.));
            // tf::Transform t2(t.getRotation(), tf::Vector3(0., 0.,0.));


            // // tf::Vector3 vec = t1.inverse()*(t2.inverse()*eigenVectorToTfVector(u/scale));
            // tf::Vector3 vec = center.inverse()*eigenVectorToTfVector(u/scale);

            // Eigen::VectorXd u_rev = tfVectorToEigenVector(vec);
            // x_new[3] = (x[3] + (revolute->JacobianInverse(x.tail(1), 1e-6)*u_rev)[0]);

            // std::cout << "\nMethod 2: " << std::endl ;
            // std::cout << "Center Transform: \nCenter:" << PRINT_TRANSFORM(center) << std::endl;
            // std::cout << "Input Vector: " << u.transpose() << std::endl;
            // std::cout << "Vector in Center Frame with LA: " << u_rev.transpose() << std::endl;

            // tf::Vector3 vec2 = t1.inverse()*eigenVectorToTfVector(u/scale);
            // Eigen::VectorXd u_rev2 = tfVectorToEigenVector(vec2);

            // std::cout << "Vector in Center Frame w/o LA: " << u_rev2.transpose() << std::endl;

            // std::cout << "Input Angle: " << x.tail(1) << std::endl;
            // std::cout << "Jacobian Inverse: " << revolute->JacobianInverse(x.tail(1), 1e-4) << std::endl;
            // std::cout << "New Angle: " << x_new[3] << std::endl;


            // // Free body part
            // geometry_msgs::Pose pose_ = revolute->forwardKinematics(x_new.tail(1));
            // tf::Transform new_pose = ref_frame*poseToTransform(pose_); // Transforming it to the world frame 
            // x_new.head(3) = scale*tfVectorToEigenVector(new_pose.getOrigin());

            // std::cout << "New Pose: " << PRINT_TRANSFORM(new_pose) << std::endl;
            // std::cout << "x_new: " << x_new.transpose() << std::endl;
            

        }

        void getObservation(Eigen::VectorXd x, Eigen::VectorXd& z)
        {
            Eigen::VectorXd z_free;
            freebody->getObservation(x.head(3), z_free);
            z.head(3)  = z_free;

            // Eigen::VectorXd z_rev;
            // revolute->getObservation(x.tail(1), z_rev);
            // z.tail(1) = z_rev;
            z[3]  = x[3];
        }

        Eigen::MatrixXd V_mat() {
            return 0.02 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
        }

        Eigen::MatrixXd W_mat() {
            return 0.005 * (scale * scale) * Eigen::MatrixXd::Identity(4, 4);
        }

};
}
