#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <changepoint_detection/articulation_models/rotational_model.h>

namespace visualize{

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

    // Function for publishing axes
    visualization_msgs::Marker prepare_axis_marker(
                        tf::Vector3 base_pt, 
                        tf::Vector3 axes,
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=100,
                        std::vector<double> color={1.0, 0., 0., 1.0},
                        std::vector<double> scale={0.25, 0.05, 0.05}){
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame;
        marker.header.stamp = ros::Time::now();

        // Specify properties
        marker.ns = "/";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = base_pt.getX();
        marker.pose.position.y = base_pt.getY();
        marker.pose.position.z = base_pt.getZ();

        tf::Vector3 x_axis(1., 0., 0.), y_axis(0., 1., 0.), z_axis(0.,0., 1.);
        float pitch = x_axis.angle(axes);
        float yaw = y_axis.angle(axes);
        float roll = z_axis.angle(axes);

        tf::Quaternion quat = tf::Quaternion(yaw, pitch, roll).normalized();
        geometry_msgs::Quaternion gm_quat;
        tf::quaternionTFToMsg(quat, gm_quat);

        std::cout << "Angle wrt X-axis: " << pitch << std::endl;
        std::cout << "Angle wrt Y-axis: " << yaw << std::endl;
        std::cout << "Angle wrt Z-axis: " << roll << std::endl;
        std::cout << "New Quat: " << gm_quat << std::endl;

        tf::Quaternion quat2(0.995, 0.088, 0.014, 0.045);
        tf::Quaternion quat3 = quat*quat2;
        std::cout << "Angle check: " << quat3.getAngle() << std::endl;

        marker.pose.orientation = gm_quat;
        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];


        return marker;
    }

    visualization_msgs::Marker prepare_axis_marker(
                        tf::Vector3 base_pt, 
                        tf::Quaternion axes,
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=200,
                        std::vector<double> color={1.0, 0., 0., 1.0},
                        std::vector<double> scale={0.25, 0.05, 0.05}){

        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame;
        marker.header.stamp = ros::Time::now();

        // Specify properties
        marker.ns = "/";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = base_pt.getX();
        marker.pose.position.y = base_pt.getY();
        marker.pose.position.z = base_pt.getZ();

        geometry_msgs::Quaternion gm_quat;
        tf::quaternionTFToMsg(axes.normalized(), gm_quat);

        marker.pose.orientation = gm_quat;
        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        return marker;
    }


    // function for publishing circular sector
    visualization_msgs::Marker prepare_sector_marker(
                        tf::Vector3 base_pt, 
                        tf::Quaternion axes,
                        std::vector<geometry_msgs::Point>& pts,
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=300,
                        std::vector<double> color={0., 1., 0., 0.75},
                        std::vector<double> scale={0.05, 0.05, 0.05}){

        // Create Markers
        visualization_msgs::Marker marker;
        marker.header.frame_id = ref_frame;
        marker.header.stamp = ros::Time::now();

        // Specify properties
        marker.ns = "/";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        // marker.pose.position.x = base_pt.getX();
        // marker.pose.position.y = base_pt.getY();
        // marker.pose.position.z = base_pt.getZ();
        marker.pose.position.x = 0.;
        marker.pose.position.y = 0.;
        marker.pose.position.z = 0.;

        // geometry_msgs::Quaternion gm_quat;
        // tf::quaternionTFToMsg(axes.normalized(), gm_quat);
        // marker.pose.orientation = gm_quat;
        marker.pose.orientation.x = 0.;
        marker.pose.orientation.y = 0.;
        marker.pose.orientation.z = 0.;
        marker.pose.orientation.w = 1.;

        marker.scale.x = scale[0];
        marker.scale.y = scale[1];
        marker.scale.z = scale[2];
        
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        marker.points = pts;

        return marker;
    }

    visualization_msgs::Marker generate_sector(
                        tf::Vector3 center, 
                        tf::Quaternion axes,
                        double rad,
                        tf::Quaternion ori,
                        geometry_msgs::Pose start_pose,
                        geometry_msgs::Pose end_pose,
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=300,
                        std::vector<double> color={0., 1., 0., 1.0},
                        std::vector<double> scale={0.05, 0.05, 0.05}){
        articulation_models::RotationalModel rot;        
        rot.rot_center = center;
        rot.rot_axis = axes;
        rot.rot_radius = rad;
        rot.rot_orientation = ori;

        std::vector<geometry_msgs::Point> pts;

        articulation_models::V_Configuration q, q_end;
        
        q = rot.predictConfiguration(start_pose);
        q_end = rot.predictConfiguration(end_pose);

        geometry_msgs::Pose new_pt;
        geometry_msgs::Point center_pt;
        center_pt.x = center.getX();
        center_pt.y = center.getY();
        center_pt.z = center.getZ();

        while(q(0) <= q_end(0)){
            pts.push_back(center_pt);
            new_pt = rot.predictPose(q);
            pts.push_back(new_pt.position);
            q(0) += 0.025;
        }

        visualization_msgs::Marker marker = visualize::prepare_sector_marker(center, axes, pts, ref_frame, marker_id, color, scale);
        return marker;
    }

    visualization_msgs::MarkerArray generate_sector_array(
                        tf::Vector3 center, 
                        tf::Quaternion axes,
                        double rad,
                        tf::Quaternion ori,
                        geometry_msgs::Pose start_pose,
                        geometry_msgs::Pose end_pose,
                        std::string ref_frame="linear_actuator_link",
                        int marker_id=300,
                        std::vector<double> color={0., 1., 0., 1.0},
                        std::vector<double> scale={0.05, 0.05, 0.05}){

        visualization_msgs::MarkerArray m_array;
        visualization_msgs::Marker m = visualize::generate_sector(center, axes, rad, ori, start_pose, end_pose, ref_frame, marker_id, color, scale);
        m_array.markers.push_back(m);

        std::vector<double> c1 = {1.0, 0., 0., 1.};
        visualization_msgs::Marker ax = visualize::prepare_axis_marker(center, axes, "odom", marker_id+1, c1);
        m_array.markers.push_back(ax);

        std::vector<double> c2 = {0., 1.0, 0., 1.};
        visualization_msgs::Marker orio = visualize::prepare_axis_marker(center, ori, "odom", marker_id+2, c2);
        m_array.markers.push_back(orio);

        return m_array;
    }

} // End namespace