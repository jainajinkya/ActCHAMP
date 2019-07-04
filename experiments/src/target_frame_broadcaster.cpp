/* Node to broadcast the target frame on tf  */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <experiments/utils.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_frame_broadcaster");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    std::string leader = "right" ;
    std::string follower = "left" ;

    ros::Rate rate(20.0);

    while(nh.ok()){
        tf::StampedTransform trans_leader, trans_target;

        // Listen to the transform
        try{
          listener.lookupTransform(leader+"_link_base", leader+"_ee_link", 
                                   ros::Time(0), trans_leader);
          // utils::printTFStampedTransform(trans_leader);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          rate.sleep();
          continue;
        }

        // Broadcast the target frame
        trans_target = trans_leader;
        trans_target.frame_id_ = follower + "_link_base";
        trans_target.child_frame_id_ = "target_frame";
        trans_target.stamp_ = ros::Time::now();

        // utils::printTFStampedTransform(trans_target);
        br.sendTransform(trans_target);

        rate.sleep();
    }

    return 0;
};