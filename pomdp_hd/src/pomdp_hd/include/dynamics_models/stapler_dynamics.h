#ifndef STAPLER_DYNAMICS_H_
#define STAPLER_DYNAMICS_H_

#include "revolute_pair.h"
#include "freebody_pair.h"
#include "kp_utils.h"


namespace dynamics {

class StaplerDynamics: public Dynamics {
public:
    std::shared_ptr<RevolutePair> revolute;
    std::shared_ptr<FreebodyPair> freebody;

    StaplerDynamics(int n_x_rev, int n_u_rev, int n_z_rev, int n_x_free, int n_u_free, int n_z_free)
    {
        nState = n_x_rev + n_x_free;
        nInput = 3;
        nOutput = n_z_rev + n_z_free;

        revolute = std::make_shared<dynamics::RevolutePair>(dynamics::RevolutePair(n_x_rev, n_u_rev, n_z_rev));
        freebody = std::make_shared<dynamics::FreebodyPair>(dynamics::FreebodyPair(n_x_free, n_u_free, n_z_free));

    }

    ~StaplerDynamics() {}

    // Variables
    // double scale = 1; // Scaling up dimensions for planning
    double theta_fixed = -2.22*scale;
    double theta_max = -1.12*scale;
    double theta_flat = -1.75;  

    tf::Transform ref_frame_;
    double action_scale;

    // Obstacles
    double y_obs = 0.05*scale;
    double z_obs = -0.05*scale;
    double z_obs_top = 0.38*scale;


    // Functions
    void printModelParams()
    {
        revolute->printModelParams();
        freebody->printModelParams();
    }

    size_t getDOFs() {return revolute->nState + freebody->nState;}

    void set_planning_scale(double val) {
        scale = val;
        revolute->scale = val;
        freebody->scale = val;
    }

    virtual void propagateState(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&) = 0;
    virtual void propagateStateUseTransform(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0))) = 0;

    // virtual void propagateState(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd&, tf::Transform t = tf::Transform(tf::Quaternion(-0.6603, 0.0848,0.7410,0.0822).normalized(), tf::Vector3(0.,0.,0))) = 0;

    virtual void getObservation(Eigen::VectorXd, Eigen::VectorXd&) = 0;

    void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd&, Eigen::MatrixXd&);
    void propagateStateWithCov(Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd&, Eigen::MatrixXd&, tf::Transform t = tf::Transform(tf::Quaternion(0., 0., 0., 1.), tf::Vector3(0.,0.,0)));
    void getObservationNoisy(Eigen::VectorXd, Eigen::VectorXd&);

    // System Matrices
    Eigen::MatrixXd A_mat(Eigen::VectorXd, Eigen::VectorXd, double delta = 1e-6);
    Eigen::MatrixXd B_mat(Eigen::VectorXd, Eigen::VectorXd, double delta = 1e-6);
    Eigen::MatrixXd C_mat(Eigen::VectorXd, double delta = 1e-6);
    virtual Eigen::MatrixXd V_mat() = 0;
    virtual Eigen::MatrixXd W_mat() = 0;


    // Utilities
    // Eigen::VectorXd predictStaplerSecondArmCoords(V_Configuration);

    // Redundent Functions
    geometry_msgs::Pose forwardKinematics(Eigen::VectorXd){
        geometry_msgs::Pose pose_;
        return pose_;
    }

    Eigen::VectorXd inverseKinematics(geometry_msgs::Pose){
        Eigen::VectorXd q(1);
        q << 0.;
        return q;
    };

    bool set_ref_called = false;

    tf::Transform getRefFrame()
    {
        std::cout << "stapler dynamics vala getRef : " << set_ref_called << std::endl;
        if(!set_ref_called)
            return tf::Transform(tf::Quaternion(0.,0., 0., 1.), tf::Vector3(0.,0.,0.));
        else
        {
            std::cout << "ref_frame_ : " <<  PRINT_TRANSFORM(ref_frame_) << std::endl;
            return ref_frame_;
        }
    }

    void setRefFrame(tf::Transform t)
    {
        ref_frame_.setOrigin(t.getOrigin());
        ref_frame_.setRotation(t.getRotation());
        set_ref_called = true;
    }



};
}

#endif // STAPLER_DYNAMICS_H_
