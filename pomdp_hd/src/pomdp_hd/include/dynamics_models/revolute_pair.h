#ifndef REVOLUTE_PAIR_H_
#define REVOLUTE_PAIR_H_

#include "generic_kinematic_pair.h"

namespace dynamics{

class RevolutePair : public GenericKinematicPair{
    public:
        RevolutePair(int n_x, int n_u, int n_z);
        ~RevolutePair(){}
    
        tf::Vector3 rot_center;
        tf::Quaternion rot_axis;    
        double rot_radius;    
        tf::Quaternion rot_orientation;
        
        // Functions
        void printModelParams();
        
        size_t getDOFs() {return nState;}
        geometry_msgs::Pose forwardKinematics(V_Configuration q);
    	V_Configuration inverseKinematics(geometry_msgs::Pose pose);

        void propagateState(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&);
        void propagateStateUseTransform(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0)));
        void getObservation(Eigen::VectorXd, Eigen::VectorXd&);
};
}

#endif // REVOLUTE_PAIR_H_
