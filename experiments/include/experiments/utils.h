#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf2/convert.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

// #include <Eigen/Core>
// #include <Eigen/Householder>
// #include <Eigen/Geometry>



namespace tf2
{
    inline
    void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
    {
        pose.orientation = trans.rotation;
        pose.position.x = trans.translation.x;
        pose.position.y = trans.translation.y;
        pose.position.z = trans.translation.z;
    }

    inline
    void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
      {
        trans.rotation = pose.orientation;
        trans.translation.x = pose.position.x;
        trans.translation.y = pose.position.y;
        trans.translation.z = pose.position.z;
    }

    inline
    void convert(const tf::Vector3& vec, tf::Point& pt)
    {
        pt.setX(vec.getX());
        pt.setY(vec.getY());
        pt.setZ(vec.getZ());

    }

    inline
    void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
    {
        convert(trans.transform, pose.pose);
        pose.header = trans.header;
    }

    inline
    void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
    {
        convert(pose.pose, trans.transform);
        trans.header = pose.header;
    }
}


namespace utils
{
    inline 
    void printTFVector(tf::Vector3 vec)
    {
        std::cout << "X: " <<  vec.getX() <<  "\tY: " <<  vec.getY() << "\tZ: " << vec.getZ() << std::endl;
        return;
    }

    inline
    void printTFMatrix(tf::Matrix3x3 mat)
    {
        std::cout << "Vector 1: " << mat.getColumn(0).getX() <<  "," <<  mat.getColumn(0).getY() << ", " <<  mat.getColumn(0).getZ() << std::endl;
        std::cout << "Vector 2: " << mat.getColumn(1).getX() <<  "," <<  mat.getColumn(1).getY() << ", " <<  mat.getColumn(1).getZ() << std::endl;
        std::cout << "Vector 3: " << mat.getColumn(2).getX() <<  "," <<  mat.getColumn(2).getY() << ", " <<  mat.getColumn(2).getZ() << "\n" << std::endl;
        return;
    }


    inline
    void printTFTransform(tf::Transform trans){
        tf::Vector3 vec = trans.getOrigin();
        utils::printTFVector(vec);
        tf::Matrix3x3 mat = trans.getBasis();
        utils::printTFMatrix(mat);
    }

    inline 
    void printTFStampedTransform(tf::StampedTransform s_trans)
    {   
        std::cout << "frame_id_ : " << s_trans.frame_id_ << std::endl;
        std::cout << "child_frame_id_ : " << s_trans.child_frame_id_ << std::endl;
        std::cout << "stamp_ : " << s_trans.stamp_ << std::endl;

        tf::Transform trans;
        trans.setOrigin(s_trans.getOrigin());
        trans.setBasis(s_trans.getBasis());
        utils::printTFTransform(trans);
    }


    inline
    float thresholded_to_zero(float val){
        if(val < 1e-15)
            return 0.;}


    inline  
    tf::Matrix3x3 generateBasis(tf::Vector3 vec_x)
    {
        if(!vec_x.isZero()){
            tf::Vector3 vec_y, vec_z;

            vec_x.normalize();          
            vec_y = tf::Vector3(0., 1., 0.);

            if(vec_x.dot(vec_y) > 0.95)
                vec_y = tf::Vector3(0., 0., 1.);          

            vec_y -= (vec_x.dot(vec_y))*vec_x; // removing component along vec_x
            vec_y.normalize();
            vec_z = (vec_x.cross(vec_y)).normalized();            

            // std::cout << "\n\nvec_x: \t";
            // utils::printTFVector(vec_x);
            // std::cout << "vec_y: \t";
            // utils::printTFVector(vec_y);
            // std::cout << "vec_z: \t";
            // utils::printTFVector(vec_z);

            // tf::
            tf::Matrix3x3 basis(vec_x[0], vec_x[1], vec_x[2], 
                                vec_y[0], vec_y[1], vec_y[2], 
                                vec_z[0], vec_z[1], vec_z[2]);
            return basis;
        }
        else{
            return tf::Matrix3x3::getIdentity();
        }

    }

}