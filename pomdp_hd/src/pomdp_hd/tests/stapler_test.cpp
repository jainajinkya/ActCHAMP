#include "../include/dynamics_models/stapler_dynamics.h"
#include "../src/dynamics_models/stapler/stapler_dynamics_A.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_B.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_C.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_D.cpp"



int main()
{
    dynamics::StaplerDynamicsA staplerA(1, 1, 1, 3, 3, 3);
    dynamics::StaplerDynamicsB staplerB(1, 1, 1, 3, 3, 3);
    dynamics::StaplerDynamicsC staplerC(1, 1, 1, 3, 3, 3);
    dynamics::StaplerDynamicsD staplerD(1, 1, 1, 3, 3, 3);


    std::cout << "Stapler object constructed!" << std::endl;

    /* SETUP */
    auto rev = std::make_shared<dynamics::RevolutePair> (dynamics::RevolutePair(1, 1, 1));

    // tf::Quaternion rot_offset = tf::Quaternion(0., 0.,-0.38, 0.925);
    tf::Quaternion rot_offset = tf::Quaternion(0., 0., 0., 1.);

    rev->rot_radius = 0.159482;
    rev->rot_center = tf::Vector3 (-0.008719, -0.053423, -0.039876);
    rev->rot_axis = tf::Quaternion(-0.180677, 0.092525, -0.469474, 0.859296)*rot_offset;
    rev->rot_orientation = tf::Quaternion (0.059868, 0.089554, 0.824675, 0.555254);
    rev->scale = 100.;

    staplerA.revolute = rev;
    staplerA.set_planning_scale(rev->scale);

    staplerB.revolute = rev;
    staplerB.set_planning_scale(rev->scale);
    
    staplerC.revolute = rev;
    staplerC.set_planning_scale(rev->scale);
    
    staplerD.revolute = rev;
    staplerD.set_planning_scale(rev->scale);

    // stapler.printModelParams();

    // dynamics::V_Configuration q(1), q1(1);
    // geometry_msgs::Pose p;
    // q << 50.;
    // p =  rel.forwardKinematics(q);
    // q1 = rel.inverseKinematics(p);
    
    // std::cout << "Predcited Pose: " << p  << std::endl;
    // std::cout << "Predcited Configuration: " << q1  << std::endl;

    // /* State Propagation */
    Eigen::VectorXd x(4), u(3), x_new(4), x_target(4);
    x << 0., -10., 7.2, 157.;
    u << 0., 0., 0.;
    x_target << 0., -10., 0.0, 90.;

    // /* Testing if dynamics can be used for porpagation */
    int T = 5;
    u[2] = (x[2] - x_target[2])/T;

    tf::Transform t = tf::Transform(tf::Quaternion(-0.6603, 0.0848,0.7410,0.0822).normalized(), tf::Vector3(0.,0.,0));

    // for(uint i=0; i<T; i++)
    // {
    //     staplerB.propagateState(x, u, x_new, t);

    //     std::cout << "\nTime step: " << i << std::endl;
    //     std::cout << "x : \t" << x.transpose() << std::endl;
    //     std::cout << "u : \t" << u.transpose() << std::endl;
    //     std::cout << "x_new : " << x_new.transpose() << std::endl;

    //     x = x_new;
    // }

    // std::cout << "\nFinal State: " << x.transpose()  << "\n\n"<< std::endl;

    // /* Testing Dynamics D */
    x << 0., -10., 0.0, 65.;
    u[2] = 15.9482*std::sin((x[3]/100.))/T ;

    for(uint i=0; i<T; i++)
    {
        staplerD.propagateStateUseTransform(x, u, x_new, t);

        std::cout << "\nTime step: " << i << std::endl;
        std::cout << "x : \t" << x.transpose() << std::endl;
        std::cout << "u : \t" << u.transpose() << std::endl;
        std::cout << "x_new : " << x_new.transpose() << std::endl;

        x = x_new;
    }

    std::cout << "\nFinal State: " << x.transpose() << std::endl;

    // /* Testing if Dynamics Matrices are set correctly */
    // std::cout << "For Dynamics A:\n" << std::endl;
    // std::cout << "A_mat: " << staplerA.A_mat(x, u) << std::endl;
    // std::cout << "B_mat: " << staplerA.B_mat(x, u) << std::endl;
    // std::cout << "C_mat: " << staplerA.C_mat(x) << std::endl;

    // std::cout << "\n\nFor Dynamics B:\n" << std::endl;
    // std::cout << "A_mat: " << staplerB.A_mat(x, u) << std::endl;
    // std::cout << "B_mat: " << staplerB.B_mat(x, u) << std::endl;
    // std::cout << "C_mat: " << staplerB.C_mat(x) << std::endl;

    //     std::cout << "\n\nFor Dynamics C:\n" << std::endl;
    // std::cout << "A_mat: " << staplerC.A_mat(x, u) << std::endl;
    // std::cout << "B_mat: " << staplerC.B_mat(x, u) << std::endl;
    // std::cout << "C_mat: " << staplerC.C_mat(x) << std::endl;


    // std::cout << "\n\nFor Dynamics D:\n" << std::endl;
    // std::cout << "A_mat: " << staplerD.A_mat(x, u) << std::endl;
    // std::cout << "B_mat: " << staplerD.B_mat(x, u) << std::endl;
    // std::cout << "C_mat: " << staplerD.C_mat(x) << std::endl;





    // x << 15.;
    // u << 10.;
    // stapler.propagateState(x, u, x_new);
    
    // staplerB.propagateState(x, u, x_new);
    // staplerC.propagateState(x, u, x_new);


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
