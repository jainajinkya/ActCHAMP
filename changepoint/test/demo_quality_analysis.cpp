#include <iostream>
#include <math.h>
#include "../include/changepoint/articulation.h"
#include "../include/changepoint/articulation_models/generic_model.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Pose.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


using namespace std;

geometry_msgs::Pose vectorToPose(std::vector<double>& data)
{
    geometry_msgs::Pose pos_;
    pos_.position.x = data[0];
    pos_.position.y = data[1];
    pos_.position.z = data[2];
    pos_.orientation.x = data[3];
    pos_.orientation.y = data[4];
    pos_.orientation.z = data[5];
    pos_.orientation.w = data[6];
    return pos_;
}

int main(){
    // Load Data
    std::string f_name = "/home/ajinkya/Software/ajinkya/active_champ/experiments/data/bagfiles/processed/microwave_data/mw_run_108_filtered_original.bag"; // mw_no grip

    // std::string f_name = "/home/ajinkya/Software/ajinkya/active_champ/experiments/data/bagfiles/processed/old_data/microwave_run6_filtered_original.bag"; // microwave_with_grip

    // std::string f_name = "/home/ajinkya/Software/ajinkya/active_champ/experiments/data/bagfiles/processed/drawer_data/drawer_run3_filtered_original.bag"; // drawer_no grip

    // std::string f_name = "/home/ajinkya/Software/ajinkya/active_champ/experiments/data/bagfiles/processed/old_data/drawer_openClose_3_filtered_original.bag"; // drawer_grip 
    // std::string f_name = "/home/ajinkya/Software/ajinkya/active_champ/experiments/data/bagfiles/processed/old_data/drawer_openClose_3_close_only_filtered_original.bag"; // drawer_grip    



    rosbag::Bag bag;
    bag.open(f_name, rosbag::bagmode::Read);

    std::vector<std::vector<double> > pose_data, action_data;

    rosbag::View view(bag, rosbag::TopicQuery(std::string("changepoint/processed_pose_data")));
    foreach(rosbag::MessageInstance const m, view)
    {
        changepoint::DataPoint::ConstPtr pose = m.instantiate<changepoint::DataPoint>();
        if(pose != NULL)
            pose_data.push_back(pose->point);
        else
            std::cout << "Failed to load pose data" << std::endl;
    }

    rosbag::View view1(bag, rosbag::TopicQuery(std::string("changepoint/processed_action_data")));
    foreach(rosbag::MessageInstance const m, view1)
    {
        changepoint::DataPoint::ConstPtr action = m.instantiate<changepoint::DataPoint>();
        if(action != NULL)
            action_data.push_back(action->point);
        else
            std::cout << "Failed to load action data" << std::endl;
    }
    bag.close();
    
    
    articulation_models::RotationalModel art_model;
    // cout << "Number of dofs of rotational: " << art_model.getDOFs() << endl;
   
    // // Microwave w/o grip
    art_model.rot_radius = 0.505937;
    art_model.rot_center = tf::Vector3 (0.408347462728,-0.43070113035,-0.110778859282);
    art_model.rot_axis = tf::Quaternion (-0.582976265224,-0.472297565236,0.638679360885,-0.170769897922);
    art_model.rot_orientation = tf::Quaternion (-0.202145407819,-0.631296232146,0.692759864585,-0.284052937665);
     
    
    // Microwave with grip    
    // art_model.rot_radius = 0.431572;
    // art_model.rot_center = tf::Vector3 (-0.028678, -0.29033, -0.031499);
    // art_model.rot_axis = tf::Quaternion (-0.016099, 0.6942453, -0.073193, 0.715826);
    // art_model.rot_orientation = tf::Quaternion (-0.522312, -0.453538, -0.50017, 0.520886 );
    
    
    // articulation_models::PrismaticModel art_model;  

    // // Drawer without grip
    // art_model.prismatic_dir = tf::Vector3(-0.031572, -0.094422, 0.99503);
    // art_model.rigid_position = tf::Vector3 (-0.119801, 0.357597, -0.184587);
    // art_model.rigid_orientation = tf::Quaternion (-0.0232074, 0.0581555, -0.0018167, 0.998036);
     
    // // Drawer with grip
    // art_model.prismatic_dir = tf::Vector3(0.049321, 0.000428, -0.998783);
    // art_model.rigid_position = tf::Vector3 (-0.048034, 0.368630, 0.003677);
    // art_model.rigid_orientation = tf::Quaternion (0.0046221, 0.109078, -0.000654, 0.992958);
    

    // Calculate off-axis actions
    double on_axis_actions = 0;
    double off_axis_actions = 0;
    double outlier_pts = 0;
    double thres;

   //  geometry_msgs::Pose pose1 = vectorToPose(pose_data[0]);
   //  articulation_models::V_Configuration q0 = art_model.predictConfiguration(pose1);
   // // pose1.position.x += 0.0025;
   // // pose1.position.y += 0.0025;
   //  pose1.position.z += 0.01;
   
   //  articulation_models::V_Configuration q1 = art_model.predictConfiguration(pose1);
   //  thres = abs(q1[0] - q0[0]);

    thres = 0.01;
    std::cout << "Threshold: " << thres << std::endl ;
    
    int use_method = 1;
    double avg_delta = 0.;
    int prev_idx = 0;
    int delta_t = 0;
    int max_delta_t = 0;
    double pose_err = 0;

    for(int i=0; i<pose_data.size()-1; i++)
    {
        geometry_msgs::Pose pos_0 = vectorToPose(pose_data[i]);
        geometry_msgs::Pose pos_1 = vectorToPose(pose_data[i+1]);


        // Average Error in Prediction
        tf::Transform trans_actual = articulation_models::poseToTransform(pos_0);
        tf::Transform trans_pred = articulation_models::poseToTransform(art_model.predictPose(art_model.predictConfiguration(pos_0)));
        tf::Transform diff_tf = trans_pred.inverseTimes(trans_actual);
        pose_err = diff_tf.getOrigin().length();

        if(pose_err > thres)
            outlier_pts += 1;

        // if(use_method == 1)
        // {
        //     double diff = abs(art_model.predictConfiguration(pos_1)[0] - art_model.predictConfiguration(pos_0)[0]);
        //     // std::cout << "Delta q: " << diff << std::endl;
        //     avg_delta += diff;

        //     if(diff < thres)
        //         off_axis_actions += 1;
        //     else
        //     {
        //         on_axis_actions += 1;

        //         delta_t = i - prev_idx;
        //         if(delta_t > max_delta_t)
        //         {
        //             std::cout << "Start time: " << prev_idx << "\tEnd time: " << i  << "\tDelta time: " << delta_t << std::endl;
        //             max_delta_t = delta_t;
        //         }
        //         prev_idx = i;
        //     }
        // }
        // else
        // {
        
        //     Eigen::VectorXd act(3);
        //     act << action_data[i][0], action_data[i][1], action_data[i][2];

        //     articulation_models::V_Configuration q;
        //     q = art_model.predictConfiguration(pos_0);
        //     //std::cout << "At t= " << i+1 << "  Predicted Configuration: " << q << std::endl;

        //     Eigen::MatrixXd Jac = art_model.predictJacobian(q);
        //     Eigen::VectorXd v = Jac/Jac.norm();
        //     Eigen::MatrixXd Jac_inv = Jac.transpose()/SQR(Jac.norm());
            
        //     double ang = acos(act.dot(v)/(act.norm() * v.norm()))* (180/M_PI);
        //     // std::cout << "Angle b/w v and action: " << ang << std::endl;

        //     // if(ang > 75 && ang < 105)
        //     if(abs((Jac_inv*act)[0]) < thres)
        //     {
        //         off_axis_actions += 1;
        //     }
        //     else
        //     {
        //         //std::cout << "Delta q: " << Jac_inv*act << std::endl;
        //         on_axis_actions += 1;
        //     }
            
        //     // std::cout << "Action Contribution: " << Jac_inv*act << std::endl;
        //     // std::cout << "Champ prediction: " << art_model.predictConfiguration(pos_1) << std::endl;
        //     // std::cout << "Difference: " << (q + Jac_inv*act)- art_model.predictConfiguration(pos_1) << "\n\n" << std::endl;
        // }
            
    }

    std::cout << "File name: " << f_name << std::endl;
    // std::cout << "Avg Pose Error: " << avg_pose_err/(off_axis_actions + on_axis_actions) << std::endl;  
//     // std::cout << "Avg Delta: " << avg_delta/(off_axis_actions + on_axis_actions) << std::endl;   
//     std::cout << "Max delta t = " << max_delta_t << std::endl;
    // std::cout << "# Off-Axis Actions: " << off_axis_actions << std::endl;
    // std::cout << "# On-Axis Actions: " << on_axis_actions << std::endl; 
    // std::cout << "% Off-Axis Actions: " << off_axis_actions/(off_axis_actions + on_axis_actions) << std::endl;
    // std::cout << "% On-Axis Actions: " << on_axis_actions/(off_axis_actions+ on_axis_actions) << std::endl;
    std::cout << "% noisy data: " << outlier_pts/(pose_data.size())*100 << std::endl;

}
