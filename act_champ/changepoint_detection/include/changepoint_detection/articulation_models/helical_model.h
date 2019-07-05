/*
 * helical_model.h
 *
 * Author: Ajinkya Jain
 */

#ifndef HELICAL_MODEL_H_
#define HELICAL_MODEL_H_

#include "rotational_model.h"
#include "prismatic_model.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/LinearMath/Transform.h"
#include "math.h"

namespace articulation_models {

class HelicalModel: public RotationalModel {
public:
    double hel_pitch;

    // Keeping track of rotations
    double n_rotations; 
    // int count_rotations(V_Configuration q);

    HelicalModel();

    // -- params
    void readParamsFromModel();
    void writeParamsToModel();

    size_t getDOFs() { return 1; }

    V_Configuration predictConfiguration(geometry_msgs::Pose pose);
    geometry_msgs::Pose predictPose(V_Configuration q);

//    bool guessParameters();
    bool guessParameters(geometry_msgs::Pose pose_1,geometry_msgs::Pose pose_2,geometry_msgs::Pose pose_3,geometry_msgs::Pose pose_4);
    void updateParameters(std::vector<double> delta);
    bool normalizeParameters();

};

}


#endif /* HELICAL_MODEL_H_ */
