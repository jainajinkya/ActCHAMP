#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "eigenmvn.h"
#include "utils.h"
#include "geometry_msgs/Pose.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace dynamics{

class Dynamics{
public:
    // State Space Parameters
    int nState;
    int nInput;
    int nOutput;

    // Planning Scale
    double scale = 1.;
    double integration_step_size = 1.;
    
    // params
    virtual void printModelParams() = 0;

    // tf::Transform ref_frame = tf::Transform(tf::Quaternion(0., 0., 0., 1.), tf::Vector3(0, 0, 0)); // Will need to set this up using AR-TAGs
    tf::Transform ref_frame; // Will need to set this up using AR-TAGs


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

    virtual geometry_msgs::Pose forwardKinematics(Eigen::VectorXd) = 0;
    virtual Eigen::VectorXd inverseKinematics(geometry_msgs::Pose) = 0;

    virtual tf::Transform getRefFrame() = 0;
    virtual void setRefFrame(tf::Transform t) = 0;

};

}

#endif // DYNAMICS_H_
