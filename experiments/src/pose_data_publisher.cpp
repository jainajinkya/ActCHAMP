/* Node to publish pose diff data */
#include <signal.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Transform.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
// #include <hlpr_perception_msgs/LabeledObjects.h>
#include <experiments/utils.h>
#include <experiments/visualization_utils.h>


using namespace std;

map<int, geometry_msgs::PoseStamped> ar_marker_poses;
map<string, geometry_msgs::Transform> object_poses;

int ref_marker_id = 0;
int child_marker_id = 4;

tf::Pose ref_pose_tf, pose;
tf::StampedTransform ts;
geometry_msgs::PoseStamped pose_diff, filtered_pose_diff;
geometry_msgs::PoseStamped ref_pose, filtered_ref_pose;
geometry_msgs::PoseStamped sec_pose, filtered_sec_pose;

geometry_msgs::TransformStamped trans;
ros::Publisher pose_pub, pose_unfiltered_pub, ref_frame_pub, secondary_frame_pub, visualization_pub;

tf2_ros::Buffer* tfBuffer;

// VISUALIZATION PARAMETERS    
std::vector<double> c1{0.,1.0,0.0, 1.0}; // Color
std::vector<double> s1{0.03, 0.03, 0.03}; // Scale
int marker_id = 0;

// Hlpr_object and SimTrack
string ref_object;
string moveable_object;

string eef_name="left_ee_link";
string LA_frame="linear_actuator_link";
string camera_frame="kinect_rgb_optical_frame";

// SMA Filter
std::vector<geometry_msgs::PoseStamped> pose_diff_array, ref_pose_array, sec_pose_array;


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

void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg){
    if(msg->markers.size() > 1){
        for(int i=0; i<msg->markers.size(); i++)
            ar_marker_poses[msg->markers[i].id] = msg->markers[i].pose;

        // Publish pose diff b/w ar markers
        tf::poseMsgToTF(ar_marker_poses[ref_marker_id].pose, ref_pose_tf);
        tf::poseMsgToTF(ar_marker_poses[child_marker_id].pose, pose);
        ts.stamp_ = msg->markers[ref_marker_id].header.stamp;
        ts.frame_id_ = msg->markers[ref_marker_id].header.frame_id;
        // ts.setData(pose.inverseTimes(ref_pose_tf));
        ts.setData(ref_pose_tf.inverseTimes(pose)); // This is how it should be
        tf::transformStampedTFToMsg(ts, trans);
        tf2::convert(trans, pose_diff);
        pose_unfiltered_pub.publish(pose_diff);

        // geometry_msgs::PoseStamped filtered_pose_diff;
        simple_moving_average(pose_diff, pose_diff_array, 5, filtered_pose_diff);
        pose_pub.publish(filtered_pose_diff);

        // Reference frame
        ref_pose.header = msg->markers[ref_marker_id].header;
        ref_pose.pose = ar_marker_poses[ref_marker_id].pose; // This ensures that we pass values by id
        // ref_pose.header = msg->markers[child_marker_id].header;
        // ref_pose.pose = ar_marker_poses[child_marker_id].pose;
        simple_moving_average(ref_pose, ref_pose_array, 10, filtered_ref_pose);
        ref_frame_pub.publish(filtered_ref_pose);

        // Secondary Frame
        sec_pose.header = msg->markers[ref_marker_id].header;
        sec_pose.pose = ar_marker_poses[child_marker_id].pose;
        simple_moving_average(sec_pose, sec_pose_array, 10, filtered_sec_pose);
        secondary_frame_pub.publish(filtered_sec_pose);

    }

    else{
        // Publishing last known values
        pose_unfiltered_pub.publish(pose_diff);
        pose_pub.publish(filtered_pose_diff);
        ref_frame_pub.publish(filtered_ref_pose);
        secondary_frame_pub.publish(filtered_sec_pose);
    }
}


// void color_tracker_callback(const hlpr_perception_msgs::LabeledObjects::ConstPtr& msg){
//     if(msg->objects.size() > 1){
//         for(int i=0; i<msg->objects.size(); i++)
//             object_poses[msg->labels[i].data] = msg->objects[i].transform;

//         // Publish pose b/w detected objects
//         tf::transformMsgToTF(object_poses[ref_object], ref_pose_tf);
//         tf::transformMsgToTF(object_poses[moveable_object], pose);

//         // std::cout << "Ref Transform: \n" <<  object_poses[ref_object] << std::endl;
//         // std::cout << "Object Transform: \n" <<  object_poses[moveable_object] << std::endl;


//         ts.stamp_ = msg->header.stamp;
//         // ts.frame_id_ = msg->header.frame_id;
//         ts.frame_id_ = ref_object;

//         ts.setData(pose.inverseTimes(ref_pose_tf));
//         // ts.setData(ref_pose.inverseTimes(pose)); ## This is how it should be
//         tf::transformStampedTFToMsg(ts, trans);
//         tf2::convert(trans, pose_diff);
//         pose_unfiltered_pub.publish(pose_diff);

//         geometry_msgs::PoseStamped filtered_pose_diff;
//         simple_moving_average(pose_diff, pose_diff_array, 5, filtered_pose_diff);
//         pose_pub.publish(filtered_pose_diff);

//         // Reference frame
//         geometry_msgs::PoseStamped ref_pose_, filtered_ref_pose_;
//         // ref_pose.header = msg->markers[ref_marker_id].header;
//         // ref_pose.pose = ar_marker_poses[ref_marker_id].pose; // This ensures that we pass values by id
//         ref_pose_.header = msg->header;
//         tf2::convert(object_poses[ref_object], ref_pose_.pose);
//         simple_moving_average(ref_pose_, ref_pose_array, 10, filtered_ref_pose_);
//         ref_frame_pub.publish(filtered_ref_pose_);

//     }

//     else std::cout << "Detecting only: " << msg->objects.size() << "objects" << std::endl;
// }


// Convert Transform to the Pose message
void transforms_to_pose(tf::StampedTransform transform)
{
    // Convert transform to geometry_msgs
    tf::transformStampedTFToMsg(transform, trans);
    tf2::convert(trans, pose_diff);
    pose_unfiltered_pub.publish(pose_diff);

    // Filter pose
    geometry_msgs::PoseStamped filtered_pose_diff;
    simple_moving_average(pose_diff, pose_diff_array, 5, filtered_pose_diff);
    pose_pub.publish(filtered_pose_diff);

    // Publish other info
    geometry_msgs::PoseStamped ref_pose_, filtered_ref_pose_;
    ref_pose_.header.frame_id = transform.frame_id_;
    ref_pose_.header.stamp = transform.stamp_;

    // Prepare Pose
    ref_pose_.pose.orientation.w = 1.;

    simple_moving_average(ref_pose_, ref_pose_array, 10, filtered_ref_pose_);
    ref_frame_pub.publish(filtered_ref_pose_);
 
    // Visulaize Points
    std::string ref_frame = transform.frame_id_;

    visualization_msgs::Marker pt_marker = visualize::prepare_point_marker(filtered_pose_diff, ref_frame, marker_id, c1, s1);
    visualization_pub.publish(pt_marker);
    marker_id++;
}


/*** SimTrack ***/
void simtrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Find eef wrt ref_pose
    tf::poseMsgToTF(msg->pose, ref_pose_tf); // "kinect_rgb_optical_frame"

    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = ref_object;
    ts.setData(ref_pose_tf.inverseTimes(pose)); // This is how it should be
    tf::transformStampedTFToMsg(ts, trans);
    tf2::convert(trans, pose_diff);

    // geometry_msgs::TransformStamped rel_pose;

    // try{
    //     rel_pose =  tfBuffer->lookupTransform(ref_object, moveable_object, ros::Time(0), ros::Duration(1.0) );
    //     tf2::convert(rel_pose, pose_diff);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    pose_unfiltered_pub.publish(pose_diff);

    geometry_msgs::PoseStamped filtered_pose_diff;
    simple_moving_average(pose_diff, pose_diff_array, 5, filtered_pose_diff);
    pose_pub.publish(filtered_pose_diff);

    // Lookup Transform 
    geometry_msgs::TransformStamped camera_in_LA;
    // Reference Pose   
    geometry_msgs::PoseStamped ref_pose_in, ref_pose_, filtered_ref_pose_;
    ref_pose_in.header = msg->header;
    ref_pose_in.pose = msg->pose;

    try{
        camera_in_LA =  tfBuffer->lookupTransform(LA_frame, camera_frame, ros::Time(0), ros::Duration(1.0) );
        tf2::doTransform(ref_pose_in, ref_pose_, camera_in_LA);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    simple_moving_average(ref_pose_, ref_pose_array, 10, filtered_ref_pose_);
    ref_frame_pub.publish(filtered_ref_pose_);

    // Visulaize Points
    visualization_msgs::Marker pt_marker = visualize::prepare_point_marker(filtered_pose_diff, ref_object, marker_id, c1, s1);
    visualization_pub.publish(pt_marker);
    marker_id++;
}


void simtrack_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    tf::poseMsgToTF(msg->pose, pose);

    // Secondary Frame
    sec_pose.header = msg->header;
    sec_pose.pose = msg->pose;
    simple_moving_average(sec_pose, sec_pose_array, 10, filtered_sec_pose);
    secondary_frame_pub.publish(filtered_sec_pose); // This is wrt kinect_rgb_optical_frame.
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_data_publisher");
    ros::NodeHandle nh;
    cout << "Node Initialized" << endl;

    nh.getParam("/pose_data_publisher/reference_object", ref_object);
    nh.getParam("/pose_data_publisher/moving_object", moveable_object);

    ROS_INFO("Got ref_object: %s", ref_object.c_str());
    ROS_INFO("Got moveable_object: %s", moveable_object.c_str());
    
    signal(SIGINT, mySigintHandler);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/changepoint/pose_data", 5);
    pose_unfiltered_pub = nh.advertise<geometry_msgs::PoseStamped>("/changepoint/pose_data_unfiltered", 5);
    ref_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("/changepoint/reference_frame", 5);
    secondary_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("/changepoint/secondary_frame", 5);

    visualization_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1); // Visualization


    /*** AR TAG ***/
    // ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 10, ar_callback);

    /*** Color Object Tracker ***/
    // ros::Subscriber sub = nh.subscribe("/beliefs/labels", 10,  color_tracker_callback);

    // /*** Transforms ***/

    // ros::Rate rate(10.0);
    // while (nh.ok()){
    //     tf::StampedTransform transform;
    //     try{
    //       listener.lookupTransform("linear_actuator_link", "left_ee_link",  
    //                                ros::Time(0), transform);
    //     }
    //     catch (tf::TransformException ex){
    //       ROS_ERROR("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    //     }

    //     transforms_to_pose(transform);
    //     rate.sleep();
    // }


    /*** SimTrack ***/
    tf2_ros::Buffer tfBuff;
    tf2_ros::TransformListener lr(tfBuff);
    tfBuffer = &tfBuff;

    ros::Subscriber sub_sim2 = nh.subscribe("/simtrack/"+moveable_object, 10,  simtrack_pose_callback);
    ros::Subscriber sub_sim = nh.subscribe("/simtrack/"+ref_object, 10,  simtrack_callback);

    // Fix rate of listening. Useful for simTrack
    ros::Rate r(10.0);
    while (nh.ok()){
        ros::spinOnce();
        r.sleep();
    }

    // ros::spin();
    return 0;
}
