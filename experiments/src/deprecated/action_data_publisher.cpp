/* Node to publish action data*/

#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <experiments/utils.h>

using namespace std;
using namespace message_filters;


/**************************************************************************************/

// Global Variables
geometry_msgs::PoseStamped prev_pose, cur_pose; //, pose_diff;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::WrenchStamped force_msg, force_world_msg;

tf::Pose trans1, trans2;
tf::Vector3 basis_origin_vec_world, basis_origin_vec, f_l_linkBase, f_r_linkBase , f_l_world, f_r_world, f_diff_world, f_diff_basis, f_diff_world_fb;
tf::Matrix3x3 basis, basis_world;
tf::StampedTransform tf_trans, tf_left_link_base_to_world, tf_right_link_base_to_world, tf_left_ee_link_to_world;

tf2_ros::Buffer tfBuffer;
ros::Publisher force_pub, force_world_pub;

geometry_msgs::TransformStamped getTransform(string parent, string child)
{
    geometry_msgs::TransformStamped ts;
    while(1){
        try{ 
            ts = tfBuffer.lookupTransform(parent, child,
                                   ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }
    }
    return ts;
}


void callback(const geometry_msgs::WrenchStampedConstPtr& eef_r_msg, const geometry_msgs::WrenchStampedConstPtr& eef_l_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
    // transformStamped = getTransform("left_ee_link", "right_ee_link");
    // tf2::convert(transformStamped, pose_diff); // generate pose_diff data

    /* Force Data */
    if(prev_pose.header.stamp.toSec() <= 0.00){
        // tf2::convert(transformStamped, prev_pose);  // Setting up prev_pose
        prev_pose.header = pose_msg->header;
        prev_pose.pose = pose_msg->pose;
    } else {
        // Convert Both forces in the world frame "world == 'linear_actuator_link"
        tf::transformStampedMsgToTF(getTransform("linear_actuator_link", "right_link_base"), tf_right_link_base_to_world);
        tf::transformStampedMsgToTF(getTransform("linear_actuator_link", "left_link_base"), tf_left_link_base_to_world);

        tf::vector3MsgToTF(eef_r_msg->wrench.force, f_r_linkBase);
        tf::vector3MsgToTF(eef_l_msg->wrench.force, f_l_linkBase);

        f_r_world = tf_right_link_base_to_world*f_r_linkBase;
        f_l_world = tf_left_link_base_to_world*f_l_linkBase;
        
        // Take a difference between the components of forces
        // This way to make sure diff of force directions correspond to +ve axes of world frame
        f_diff_world = f_l_world - f_r_world; 

        // Generate the basis for delta theta
        // tf2::convert(transformStamped, cur_pose);
        cur_pose.header = pose_msg->header;
        cur_pose.pose = pose_msg->pose;

        tf::poseMsgToTF(prev_pose.pose, trans1);
        tf::poseMsgToTF(cur_pose.pose, trans2);
        tf_trans.stamp_ = prev_pose.header.stamp;
        tf_trans.setData(trans2.inverseTimes(trans1));

        basis_origin_vec = tf_trans.getOrigin(); // LHS = diff_vector: prev_pose frame
        basis = utils::generateBasis(basis_origin_vec);

        // Find components of force in the delta theta basis
        // f_diff_basis.setX(0.);
        f_diff_basis.setY(f_diff_world.dot(basis.getColumn(1)));
        f_diff_basis.setZ(f_diff_world.dot(basis.getColumn(2)));

        // Setting the force along X-axis to be based on the pose_diff
        f_diff_basis.setX(basis_origin_vec.length());

        // Convert f_diff to world frame
        tf::transformStampedMsgToTF(getTransform("linear_actuator_link", "left_ee_link"),tf_left_ee_link_to_world);
        f_diff_world_fb = tf_left_ee_link_to_world*trans1.inverse()*f_diff_basis;

        force_msg.header = prev_pose.header;
        tf::vector3TFToMsg(f_diff_world_fb, force_msg.wrench.force);
        force_pub.publish(force_msg);

        force_world_msg.header = prev_pose.header;
        tf::vector3TFToMsg(f_diff_world, force_world_msg.wrench.force);
        force_world_pub.publish(force_world_msg);

        // Update for next loop
        prev_pose = cur_pose;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_data_publisher");
    ros::NodeHandle n;
    std::cout << "Node Initialized" << std::endl;

    tf2_ros::TransformListener tfListener(tfBuffer);
    
    force_pub = n.advertise<geometry_msgs::WrenchStamped>("/changepoint/action_data", 20);
    force_world_pub = n.advertise<geometry_msgs::WrenchStamped>("/changepoint/force_world", 20);


    message_filters::Subscriber<geometry_msgs::WrenchStamped> eef_r_sub(n, "/right_arm_driver/out/tool_wrench", 1000);
    message_filters::Subscriber<geometry_msgs::WrenchStamped> eef_l_sub(n, "/left_arm_driver/out/tool_wrench", 100);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(n, "/changepoint/pose_data", 100);


    typedef sync_policies::ApproximateTime<geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), eef_r_sub, eef_l_sub, pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));


    // ros::Rate loop_rate(10);

    // utils::printTFVector(tf_trans.getOrigin());
    // utils::printTFMatrix(tf_trans.getBasis());

    ros::spin();
    return 0;

}