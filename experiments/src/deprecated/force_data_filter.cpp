#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>


ros::Publisher force_pub_r;
ros::Publisher force_pub_l;

double cma_f_r[3] = {0, 0, 0};
double cma_f_l[3] = {0, 0, 0};
int n_f_r=0, n_f_l=0;

geometry_msgs::WrenchStamped msg_new_r, msg_new_l;


double cma_filter(int n, double cma, double x)
{
    // return x - (cma + (x-cma)/(n+1)) ; 
    return cma + (x-cma)/(n+1); 

}


void force_cb_r(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    n_f_r++;
    msg_new_r.header = msg->header;
    msg_new_r.wrench.force.x = cma_filter(n_f_r, cma_f_r[0], msg->wrench.force.x);
    msg_new_r.wrench.force.y = cma_filter(n_f_r, cma_f_r[1], msg->wrench.force.y);
    msg_new_r.wrench.force.z = cma_filter(n_f_r, cma_f_r[2], msg->wrench.force.z);
    msg_new_r.wrench.torque = msg->wrench.torque;
    force_pub_r.publish(msg_new_r);
}


void force_cb_l(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    n_f_l++;
    msg_new_l.header = msg->header;
    msg_new_l.wrench.force.x = cma_filter(n_f_l, cma_f_l[0], msg->wrench.force.x);
    msg_new_l.wrench.force.y = cma_filter(n_f_l, cma_f_l[1], msg->wrench.force.y);
    msg_new_l.wrench.force.z = cma_filter(n_f_l, cma_f_l[2], msg->wrench.force.z);
    msg_new_l.wrench.torque = msg->wrench.torque;
    force_pub_l.publish(msg_new_l);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_data_filter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    force_pub_r = nh.advertise<geometry_msgs::WrenchStamped>("/right_arm_driver/out/tool_wrench_filtered", 100);
    force_pub_l = nh.advertise<geometry_msgs::WrenchStamped>("/left_arm_driver/out/tool_wrench_filtered", 100);

    ros::Subscriber sub1 = nh.subscribe("/right_arm_driver/out/tool_wrench", 100, force_cb_r);
    ros::Subscriber sub2 = nh.subscribe("/left_arm_driver/out/tool_wrench", 100, force_cb_l);

    ros::spin();
    return 0;
}