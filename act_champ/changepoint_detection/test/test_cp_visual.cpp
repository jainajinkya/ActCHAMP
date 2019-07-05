// Unit tests to check if visualization header is working

#include <changepoint_detection/visualization_utils.h>


int main( int argc, char** argv )
{
    ros::init(argc, argv, "test_visualization", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    std::string ref_frame = "linear_actuator_link";


    std::vector<double> c1{0.5,0.5,0.5, 1.0};
    std::vector<double> c2{1.0,0.,0.5, 1.0};
    std::vector<double> c3{1.0,0.5,0., 0.5};

    // Point Marker Test
    geometry_msgs::PoseStamped pos_;
    pos_.pose.position.x = 1.0;
    pos_.pose.position.y = 0.;
    pos_.pose.position.z = 0.5;
    pos_.pose.orientation.x = 0.;
    pos_.pose.orientation.y = 0.;
    pos_.pose.orientation.z = 0.;
    pos_.pose.orientation.w = 1.;
    
    visualization_msgs::Marker pt_marker = visualize::prepare_point_marker(pos_, ref_frame, 0, c1);


    // Axis Marker Test
    tf::Vector3 vec(1.0, 0., 0.7);
    tf::Vector3 ori(-1.186, -0.340, -1.048);

    if(argc > 1){
        tf::Vector3 ori_2(atof(argv[1]), atof(argv[2]), atof(argv[3]));
        ori = ori_2;
    }

    std::vector<double> s={1.0, 0.1,0.1};
    visualization_msgs::Marker axes_marker_vec = visualize::prepare_axis_marker(vec, ori, ref_frame, 1, c3, s);
  

  // Axis Marker Quat test
    tf::Vector3 vec2(1.0, 0., 0.7);
    tf::Quaternion quat(0.755, 0.407, 0.397, 0.327);
    visualization_msgs::Marker axes_marker_vec2 = visualize::prepare_axis_marker(vec2, quat, ref_frame, 2, c3,s);

    // Sector Marker
    tf::Vector3 center(-0.09505, -0.4440, 0.03572);
    tf::Quaternion axes(-0.635125, 0.106901, 0.747335, 0.163335);
    double rad = 0.2951;
    tf::Quaternion orient(0.62752, 0.402127, -0.37856, 0.5488);

    articulation_models::RotationalModel rot;        
    rot.rot_center = center;
    rot.rot_axis = axes;
    rot.rot_radius = rad;
    rot.rot_orientation = orient;

    articulation_models::V_Configuration q(1);
    q(0) = 0.1;
    geometry_msgs::Pose p_start = rot.predictPose(q);

    q(0) = 1.5;
    geometry_msgs::Pose p_end = rot.predictPose(q);

    // visualization_msgs::Marker sec_marker = visualize::generate_sector(center, axes, rad, orient, p_start, p_end, ref_frame, 3, c3);
    // visualization_msgs::Marker circle_axis = visualize::prepare_axis_marker(center, axes, ref_frame, 4, c1);
    // visualization_msgs::Marker circle_ori = visualize::prepare_axis_marker(center, orient, ref_frame, 5, c2);

    visualization_msgs::MarkerArray circ = visualize::generate_sector_array(center, axes, rad, orient, p_start, p_end, ref_frame, 30, c3);

    while(ros::ok())
    {
        // marker_pub.publish(axes_marker_vec);
        // r.sleep();
        // marker_pub.publish(axes_marker_vec2);
        // r.sleep();
        marker_pub.publish(pt_marker);
        r.sleep();

        // marker_pub.publish(sec_marker);
        // r.sleep();
        // marker_pub.publish(circle_ori);
        // r.sleep();
        // marker_pub.publish(circle_axis);
        // r.sleep();
        marker_array_pub.publish(circ);
        r.sleep();
    }

}