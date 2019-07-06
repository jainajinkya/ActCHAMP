// Unit tests to check if visualization header is working

#include <experiments/visualization_utils.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "test_visualization_pts", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::cout << "Node initialized!" << std::endl;

    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Point Marker Test
    geometry_msgs::PoseStamped pos_;
    pos_.pose.position.x = 1.0;
    pos_.pose.position.y = 0.;
    pos_.pose.position.z = 0.5;
    pos_.pose.orientation.x = 0.;
    pos_.pose.orientation.y = 0.;
    pos_.pose.orientation.z = 0.;
    pos_.pose.orientation.w = 1.;
    
    visualization_msgs::Marker pt_marker = visualize::prepare_point_marker(pos_, "odom", 0);

    std::cout <<"Trying to publish data points!" << std::endl;

    while(ros::ok())
    {
        marker_pub.publish(pt_marker);
        r.sleep();
    }


}