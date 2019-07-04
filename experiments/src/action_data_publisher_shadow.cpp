/* Node to publish action data in case of shadowed arm motion */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <experiments/utils.h>



int main(int argc , char **argv)
{
    ros::init(argc, argv, "action_data_publisher");
    ros::NodeHandle nh;
    ROS_INFO("Action data publisher (shadow) initialized");

    tf::TransformListener listener;
    ros::Publisher action_pub = nh.advertise<geometry_msgs::WrenchStamped>("/changepoint/action_data", 20);

    std::string follower = "left" ;
    std::string world_frame = "linear_actuator_link" ;

    tf::Vector3 action_rel, action_world;
    geometry_msgs::WrenchStamped action_msg;

    ros::Rate rate(10.0);

    while(nh.ok()){
        tf::StampedTransform trans_diff, follower_ee_to_world;

        // Listen to the transform
        try{
            listener.lookupTransform(follower+"_ee_link", "target_frame", 
                                    ros::Time(0), trans_diff);
            // utils::printTFStampedTransform(trans_diff);

            listener.lookupTransform(world_frame, follower+"_ee_link", 
                                    ros::Time(0), follower_ee_to_world);
            // utils::printTFStampedTransform(follower_ee_to_world);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          rate.sleep();
          continue;
        }
        

        // Convert relative transform to world frame pose
        action_rel = trans_diff.getOrigin(); // Can also find torque by checking how much the orientation has changed

        action_world = follower_ee_to_world.getBasis()*action_rel;
        // action_world = action_rel;

        // Prepare and publish the msg
        action_msg.header.stamp = ros::Time::now();
        action_msg.header.frame_id = world_frame;
        action_msg.wrench.force.x = action_world.getX();
        action_msg.wrench.force.y = action_world.getY();
        action_msg.wrench.force.z = action_world.getZ();

        action_pub.publish(action_msg);

        rate.sleep();
    }

    return 0;

}
