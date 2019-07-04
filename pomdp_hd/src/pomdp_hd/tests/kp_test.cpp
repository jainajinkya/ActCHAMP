#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"

#include "../include/dynamics_models/revolute_pair.h"
#include "../include/dynamics_models/prismatic_pair.h"
#include "../src/dynamics_models/stapler/stapler_dynamics_D.cpp"

tf::Transform refFrame;

void refFrameCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Transform t_;
    tf2::convert(msg->pose, t_);
    tf::transformMsgToTF(t_, ::refFrame);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/changepoint/reference_frame", 10, refFrameCallback);

    // Define Dynamics model
    dynamics::StaplerDynamicsD dyna(1,1,1,3,3,3);

    tf::Quaternion rot_offset = tf::Quaternion(0., 0.,-0.38, 0.925);

    dyna.revolute->rot_radius = 0.159482;
    dyna.revolute->rot_center = tf::Vector3 (-0.008719, -0.053423, -0.039876);
    dyna.revolute->rot_axis = tf::Quaternion (-0.180677, 0.092525, -0.469474, 0.859296)*rot_offset;
    dyna.revolute->rot_orientation = tf::Quaternion (0.059868, 0.089554, 0.824675, 0.555254);
    
    dyna.revolute->scale = 100.;
    dyna.set_planning_scale(100.);

    // dyna.printModelParams();

    Eigen::VectorXd x(4), u(3), x_new(4);
    // x << 0.912, -0.002, 0.0718, 1.57;
    x << 85., -4., 15., 157.;
    // final x: 0.868, 0.022, -0.059., 0.72;
    // u << 0., 0., -0.236;
    while(ros::ok())
    {       
        u << 0., 0., -0.0 ;
        std::cout << "Enter Value of displacement along z: ";
        std::cin >> u[2];

        std::cout << "Current State: " << x.transpose() << std::endl;
        std::cout << "Applied Delta: " << u.transpose() << std::endl;
        
        tf::Transform ref_frame_(tf::Quaternion(0.163059690456, 0.0304646059196, -0.119432119612, 0.026171355136), tf::Vector3(0.912560514074, -0.121007100408, -0.0699029223564));

        // std::cout << "Reference Frame passed: " << PRINT_TRANSFORM(ref_frame_) << std::endl;


        for(int i=0; i<100; i++)
        {
            // tf::Transform ref_frame_ = ::refFrame;
            // std::cout << "Reference Frame passed: " << PRINT_TRANSFORM(ref_frame_) << std::endl;

            dyna.propagateStateUseTransform(x, u/100, x_new, ref_frame_);
            // std::cout << "z: " << u[2] << "\t Delta theta:" << x_new[3]-x[3] << std::endl;

            x = x_new;
        }

        // dyna.propagateState(x, u, x_new, ref_frame_);

        std::cout << "New State: " << x_new.transpose() << std::endl;


        ros::spinOnce();

    }
    // dynamics::RevolutePair rel(1,1,1);
    
    // dynamics::PrismaticPair rel(1,1,1);

    // /* SETUP */
    // // rel.rot_center = tf::Vector3 (0,0,0);
    // // rel.rot_axis = tf::Quaternion (0.723, -0.119, -0.681, 0.022);
    // // rel.rot_radius = 1;
    // // rel.rot_orientation = tf::Quaternion (0.539,0.492,-0.459,0.506);
    // rel.rigid_position = tf::Vector3(0,0,0);
    // rel.rigid_orientation = tf::Quaternion(0., 0., 0., 1);
    // rel.prismatic_dir = tf::Vector3(1., 0., 0.);
    // rel.scale = 100.;

    // rel.printModelParams();

    // dynamics::V_Configuration q(1), q1(1);
    // geometry_msgs::Pose p;
    // q << 50.;
    // p =  rel.forwardKinematics(q);
    // q1 = rel.inverseKinematics(p);
    
    // std::cout << "Predcited Pose: " << p  << std::endl;
    // std::cout << "Predcited Configuration: " << q1  << std::endl;

    // /* State Propagation */
    // Eigen::VectorXd x(1), u(1), x_new(1);
    // x << 15.;
    // u << 10.;
    // rel.propagateState(x, u, x_new);

    // std::cout << "Applied action: " << u.transpose()  << std::endl;
    // std::cout << "Previous Configuration: " << x.transpose()  << std::endl;
    // // std::cout << "Previous Configuration: " << rel.inverseKinematics(dynamics::vectorToPose(x)) << std::endl;
    // std::cout << "New Configuration: " << x_new.transpose()  << std::endl;
    // // std::cout << "New Configuration: " << rel.inverseKinematics(dynamics::vectorToPose(x_new)) << std::endl;

    // std::cout << "\n\n#######################\n System Matrices" << std::endl;
    // std::cout << "A_mat : \n" << rel.A_mat(x, u) << std::endl;
    // std::cout << "B_mat : \n" << rel.B_mat(x, u) << std::endl;
    // std::cout << "C_mat : \n" << rel.C_mat(x) << std::endl;
    // std::cout << "V_mat : \n" << rel.V_mat() << std::endl;
    // std::cout << "W_mat : \n" << rel.W_mat() << std::endl;

}
