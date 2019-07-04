#ifndef PRISMATIC_PAIR_H_
#define PRISMATIC_PAIR_H_

#include "generic_kinematic_pair.h"

namespace dynamics{

class PrismaticPair : public GenericKinematicPair{
    public:
        PrismaticPair(int n_x, int n_u, int n_z);
        ~PrismaticPair(){}
    
        tf::Vector3 rigid_position;
        tf::Quaternion rigid_orientation;
        tf::Vector3 prismatic_dir; 
        
        // Functions
        void printModelParams();
        
        size_t getDOFs() { return 1; }
        geometry_msgs::Pose forwardKinematics(V_Configuration q);
    	V_Configuration inverseKinematics(geometry_msgs::Pose pose);
        
        void propagateState(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&);
        void propagateStateUseTransform(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0)));
        void getObservation(Eigen::VectorXd, Eigen::VectorXd&);
};
}

#endif // PRISMATIC_PAIR_H_
