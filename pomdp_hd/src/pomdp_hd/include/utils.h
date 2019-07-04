#ifndef UTILS_H_
#define UTILS_H_

#include <map>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

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

/* Custom Data Structures */
struct GC_tuple
{
    double min_val;
    double max_val;
};

struct Guards
{   
    std::vector<GC_tuple> conditions;
};


/* Utilities */
namespace utils{
void reduce_map_size(std::map<int, Eigen::MatrixXd> &, std::map<int, Eigen::MatrixXd> &, int); 
void reduce_map_size(std::map<int, std::vector<Guards> > &, std::map<int, std::vector<Guards> > &, int);

void activeModel(const Eigen::VectorXd&, int&, int, int, std::map<int, std::vector<Guards> > &);

void multiply_vectorMap_w_vector(std::map<int, Eigen::VectorXd>& , Eigen::VectorXd&, Eigen::Map<Eigen::VectorXd>&);

void multiply_matrixMap_w_vector(std::map<int, Eigen::MatrixXd>& , Eigen::VectorXd&, Eigen::Map<Eigen::MatrixXd>&);

std::map<int, unsigned int> counter(const std::vector<int>&);

void display(const std::map<int, unsigned int>&);

/* Math Utils */
double mvn_pdf(const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::MatrixXd &);

bool isPD(Eigen::MatrixXd&);

void nearestPD(Eigen::MatrixXd&, Eigen::MatrixXd&);

double eps(float);

} // end namespace


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
#endif // UTILS_H_
