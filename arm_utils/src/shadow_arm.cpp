/* Node to shadow arm motion to move the other one */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>       // std::vector
#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>


// bool safe_pose;

// bool within_threshold(double val, double min_thres, double max_thres)
// {
//     if(val>=min_thres && val <= max_thres)
//         return true;
//     else
//         false;
// }

// void force_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg)
// {
//     std::vector<bool> v;
//     double thres = 5.;

//     v.push_back(within_threshold(msg->wrench.force.x, -thres, thres));
//     v.push_back(within_threshold(msg->wrench.force.y, -thres, thres));
//     v.push_back(within_threshold(msg->wrench.force.z, -thres, thres));
    
//     safe_pose = std::all_of(v.begin(), v.end(), [](bool val) { return val; });
// }


void print_goal(std::vector<double> leader_joint_states)
{
    std::cout << "Goal: " << leader_joint_states[0] << " , " << leader_joint_states[1] << " , " << leader_joint_states[2] << " , " << leader_joint_states[3] << " , " << leader_joint_states[4] << " , " << leader_joint_states[5] << " , " <<leader_joint_states[6] << std::endl;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_shadower");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); // In case of force callback, probably need to do this 2
    spinner.start();
    ROS_INFO("Arm Shadower Node Initialized");

    /* Setup */
    std::string leader = "right_arm";
    std::string follower = "left_arm";
    std::vector<double> leader_joint_states, prev_state;
    bool plan_success, exec_success;
    bool visualize_plans = false;
    int dummy;
    ros::Rate r(0.5);

    // std::string follower_arm_force_state_topic = follower + "_driver/out/tool_wrench";
    // ros::Subscriber follower_force_sub = nh.subscribe(follower_arm_force_state_topic, 100, force_cb);

    /* MOVEIT */
    moveit::planning_interface::MoveGroup group_leader(leader);
    moveit::planning_interface::MoveGroup group_follower(follower);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;     
    // group.setPoseReferenceFrame("linear_actutor_link");
    ROS_INFO("Reference frame for leader: %s", group_leader.getPlanningFrame().c_str());
    ROS_INFO("Reference frame for follower: %s", group_follower.getPlanningFrame().c_str());

    moveit::planning_interface::MoveGroup::Plan my_plan;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    while (ros::ok())
    {
        /* Moving follower arm*/
        group_leader.getCurrentState()->copyJointGroupPositions(group_leader.getCurrentState()->getRobotModel()->getJointModelGroup(group_leader.getName()), leader_joint_states);
        // print_goal(leader_joint_states);

        group_follower.setJointValueTarget(leader_joint_states);
        plan_success = static_cast<bool>(group_follower.plan(my_plan));

        // Visualizing plans
        if (visualize_plans)
        {
            ROS_INFO("Visualizing plan 1 ");    
            display_trajectory.trajectory_start = my_plan.start_state_;
            display_trajectory.trajectory.push_back(my_plan.trajectory_);
            display_publisher.publish(display_trajectory);
            /* Sleep to give Rviz time to visualize the plan. */
            // sleep(5.0);
        }

        if(plan_success) 
            exec_success = static_cast<bool>(group_follower.asyncMove());
        else
            ROS_INFO("PLANNING FAILED");

        // /* Checking force safety*/
        // /* NEED TO THINK MORE ON THIS */
        // ROS_INFO("If pose safe? %s", safe_pose?"":"FAILED");

        // if(!safe_pose)
        // {
        //     group_follower.setJointValueTarget(prev_state);        
        //     plan_success = static_cast<bool>(group_follower.plan(my_plan));
        //     if(plan_success)
        //         exec_success = static_cast<bool>(group_follower.asyncMove());
        //     else
        //         ROS_ERROR("FAILED TO GO BACK TO ORIGINAL POSE");

        //     ROS_INFO("TAKING A BREAK TO RECOVER FROM COLLISION");
        //     print_goal(prev_state);
        //     sleep(10);
        // }

        // Next iteration
        prev_state = leader_joint_states;
        leader_joint_states.clear();
        ros::spinOnce();
        r.sleep();
    }

    spinner.stop();
    return 0;
}
