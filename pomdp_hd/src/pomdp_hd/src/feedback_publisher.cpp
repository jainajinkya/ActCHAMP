/* Node to publish observatinal feedback for POMDP-HD */
#include <signal.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <../include/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <pomdp_hd/DataPoint.h>
#include "../include/dynamics_models/revolute_pair.h"

using namespace std;

map<int, double> observe, observe_combined;
geometry_msgs::PoseStamped sec_guess_for_ref_frame;
std::vector<geometry_msgs::PoseStamped> xyz_poses, q_poses, sec_poses;

ros::Publisher obs_pub, obs_pub_2;

// Revolute object
dynamics::RevolutePair revolute(1,1,1);


// // Tf linstener
// tf::Transform stapler_to_LA_transform(tf::Quaternion(0.219, -0.033, 0.560), tf::Vector3(0.219, -0.033, 0.560));
// string global_ref_frame = "linear_actuator_link";
// string stapler_ref_frame = "kinect_ir_optical_frame";


void mySigintHandler(int sig){
    cout << "Ctrl+C Pressed. Killing Node" << endl;
    ros::shutdown();
}


void find_average(std::vector<geometry_msgs::PoseStamped>& array,
                geometry_msgs::PoseStamped& avg_pose, int avg_len)
{
    // Function to find average of an array of poseStamped
    avg_pose = *array.begin();

    if (array.size() > 1){
        for (unsigned int i=1; i<array.size(); i++){
            avg_pose.pose.position.x += array.at(i).pose.position.x;
            avg_pose.pose.position.y += array.at(i).pose.position.y;
            avg_pose.pose.position.z += array.at(i).pose.position.z;
            avg_pose.pose.orientation.x += array.at(i).pose.orientation.x;
            avg_pose.pose.orientation.y += array.at(i).pose.orientation.y;
            avg_pose.pose.orientation.z += array.at(i).pose.orientation.z;
            avg_pose.pose.orientation.w += array.at(i).pose.orientation.w;
            // std::cout << "POSE DATAPOINT: \n" << array.at(i).pose << std::endl;
        }

        avg_pose.pose.position.x /= array.size();
        avg_pose.pose.position.y /= array.size();
        avg_pose.pose.position.z /= array.size();
        avg_pose.pose.orientation.x /= array.size();
        avg_pose.pose.orientation.y /= array.size();
        avg_pose.pose.orientation.z /= array.size();
        avg_pose.pose.orientation.w /= array.size();
        avg_pose.header = array.back().header; // last element
    }
}

void simple_moving_average(geometry_msgs::PoseStamped& pose,
                std::vector<geometry_msgs::PoseStamped>& arr, 
                int avg_len, geometry_msgs::PoseStamped& pose_avg)
{
    if(arr.size() >= avg_len)
        arr.erase(arr.begin());
        
    arr.push_back(pose);
    find_average(arr, pose_avg, avg_len);
    // std::cout << "AVERAGE POSE : \n" << pose_avg.pose << std::endl;
}

void xyz_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        // std::cout << "Input to XYZ "  << msg->pose << std::endl;
        geometry_msgs::PoseStamped pose, filtered_pose;
        pose.header= msg->header;
        pose.pose= msg->pose;

        simple_moving_average(pose, xyz_poses, 5, filtered_pose);

        observe[0] = filtered_pose.pose.position.x;
        observe[1] = filtered_pose.pose.position.y;
        observe[2] = filtered_pose.pose.position.z;


        // geometry_msgs::TransformStamped filtered_pose_gm_tf_;
        // tf::StampedTransform filtered_pose_tf, stapler_to_LA_transform_tf, global_pose;

        // tf2::convert(filtered_pose, filtered_pose_gm_tf_);
        // tf::transformStampedMsgToTF(filtered_pose_gm_tf_, filtered_pose_tf);
        
        // global_pose.setData(stapler_to_LA_transform*filtered_pose_tf); 

        // observe[0] = global_pose.getOrigin().getX();
        // observe[1] = global_pose.getOrigin().getY();
        // observe[2] = global_pose.getOrigin().getZ();
}


void theta_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // msg is the relative pose
    dynamics::V_Configuration q = revolute.inverseKinematics(msg->pose);

    geometry_msgs::PoseStamped fake_pose, filtered_q;
    fake_pose.pose.position.x = q[0];
    simple_moving_average(fake_pose, q_poses, 5, filtered_q);

    // std::cout << "Input Relative Pose: " << msg->pose << std::endl;

    dynamics::V_Configuration q_new(1);
    q_new[0] = filtered_q.pose.position.x;

    std::cout << "Filtered Angle: " << q << std::endl;
    std::cout << "Actual Angle: " << q_new << std::endl;

    observe[3] = q_new[0];

    /* Second Guess Condtion*/
    // if(q_new[0] < 0.80){
    //     // Assuming both sensory data is equally corrupt
    //     observe_combined[0] = 0.75*observe[0] + 0.25*sec_guess_for_ref_frame.pose.position.x;
    //     observe_combined[1] = 0.75*observe[1] + 0.25*sec_guess_for_ref_frame.pose.position.y;
    //     observe_combined[2] = 0.75*observe[2] + 0.25*sec_guess_for_ref_frame.pose.position.z;
    //     observe_combined[3] = q_new[0];
    // }
    // else
        // observe_combined = observe;

    observe_combined = observe; 

    std::vector<double> v, v_com;

    for(int i=0; i<4; i++)
        v.push_back(observe[i]);

    pomdp_hd::DataPoint obs;
    obs.point = v;

    v.clear();
    // Publish Data
    obs_pub.publish(obs);

    // Combined Data
    for(int i=0; i<4; i++)
        v_com.push_back(observe_combined[i]);

    pomdp_hd::DataPoint obs_com;
    obs_com.point = v_com;

    v_com.clear();
    // Publish Data
    obs_pub_2.publish(obs_com);
}

void secondary_frame_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    tf::Transform offset(tf::Quaternion(0,0,0,1), tf::Vector3(0.205, -0.025, 0.));

    tf::Transform sec_frame_tf, guess_from_sec_tf;
    geometry_msgs::Transform sec_frame_msg, guess_from_sec_msg;

    geometry_msgs::PoseStamped sec_pose, sec_pose_filtered;
    sec_pose.header = msg->header;
    sec_pose.pose = msg->pose;
    simple_moving_average(sec_pose, sec_poses, 10, sec_pose_filtered);

    tf2::convert(msg->pose, sec_frame_msg);
    tf::transformMsgToTF(sec_frame_msg, sec_frame_tf);

    guess_from_sec_tf = sec_frame_tf*offset;

    sec_guess_for_ref_frame.header = msg->header;
    tf::transformTFToMsg(guess_from_sec_tf, guess_from_sec_msg);
    tf2::convert(guess_from_sec_msg, sec_guess_for_ref_frame.pose);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "feedback_publisher");
    ros::NodeHandle nh;
    cout << "Node Initialized" << endl;

    // tf::Quaternion rot_offset = tf::Quaternion(0., 0., -0.38, 0.925);
    tf::Quaternion rot_offset = tf::Quaternion(0.0, 0., 0., 1.);


    revolute.rot_radius = 0.058711;
    revolute.rot_center = tf::Vector3 (0.01095, 0.03288, 0.04342);
    // revolute.rot_axis = tf::Quaternion (-0.13326, -0.64536, 0.295487, 0.691688)*rot_offset; // Original
    revolute.rot_axis = tf::Quaternion (-0.13326, -0.64536, 0.295487, 0.691688);
    revolute.rot_orientation = tf::Quaternion (-0.65231, 0.016179, 0.726686, -0.21482)*rot_offset;
    revolute.scale = 1.;

    cout << "Revolte Set " << endl;

    signal(SIGINT, mySigintHandler);

    ros::Subscriber sub = nh.subscribe("/changepoint/reference_frame", 5,  xyz_callback);
    // ros::Subscriber sub = nh.subscribe("/changepoint/secondary_frame", 5,  xyz_callback);


    ros::Subscriber sub_theta = nh.subscribe("/changepoint/pose_data", 5,  theta_callback);

    ros::Subscriber sec_sub = nh.subscribe("/changepoint/secondary_frame", 5,  secondary_frame_callback);

    obs_pub = nh.advertise<pomdp_hd::DataPoint>("/changepoint/observation_data", 5);
    obs_pub_2 = nh.advertise<pomdp_hd::DataPoint>("/changepoint/observation_data_combined", 5);


    // tf::TransformListener listener;

    // while (nh.ok())
    // {
    //     try{
    //       listener.lookupTransform(global_ref_frame, stapler_ref_frame,  
    //                                ros::Time(0), stapler_to_LA_transform);
    //     }
    //     catch (tf::TransformException ex){
    //       ROS_ERROR("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    //     }

    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;
}
