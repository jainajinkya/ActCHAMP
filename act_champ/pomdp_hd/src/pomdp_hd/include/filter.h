#ifndef FILTER_H_
#define FILTER_H_

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include "problem_definition.h"

namespace filters
{
class Filter : public ProblemDefinition
{
    public:
        // tf::Transform s_to_LA = tf::Transform(tf::Quaternion(0.,0.,0.,1.), tf::Vector3(0.,0.,0));
        tf::Transform s_to_LA = tf::Transform(tf::Quaternion(-0.6603, 0.0848,0.7410,0.0822).normalized(), tf::Vector3(0.,0.,0));
        

        // Methods
        virtual void prediction(Eigen::VectorXd, Eigen::MatrixXd, 
                Eigen::VectorXd,  
                Eigen::VectorXd&, Eigen::MatrixXd&) = 0;
        virtual void update(Eigen::VectorXd, Eigen::MatrixXd, 
                Eigen::VectorXd,  
                Eigen::VectorXd&, Eigen::MatrixXd&) = 0;

};
} // end filters namespace

#endif // FILTER_H_
