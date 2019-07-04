// Header file for helper visualization functions
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

namespace visualize{

// Function for publishing points
    // Function for publishing Rigid
    visualization_msgs::Marker prepare_point_marker(
                        geometry_msgs::PoseStamped pos_, 
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=0,
                        std::vector<double> color={1.0, 0., 0., 0.5},
                        std::vector<double> scale={0.5, 0.5, 0.5}){
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame;
        marker.header.stamp = ros::Time::now();

        // Specify properties
        marker.ns = "/";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pos_.pose;
        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3]; // Don't forget to set the alpha!

        return marker;
    }

} // end namespace