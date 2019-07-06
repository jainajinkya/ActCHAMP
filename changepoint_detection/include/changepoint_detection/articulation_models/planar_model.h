/*
 * planar_model.h
 *
 *  Created on: Oct 10, 2018
 *      Author: Ajinkya
 */

#ifndef PLANAR_MODEL_H_
#define PLANAR_MODEL_H_

#include "rigid_model.h"

namespace articulation_models {

class PrismaticModel: public RigidModel {
public:
	tf::Vector3 prismatic_dir;

	PrismaticModel();

	// -- params
	void readParamsFromModel();
	void writeParamsToModel();

	size_t getDOFs() { return 1; }

	V_Configuration predictConfiguration(geometry_msgs::Pose pose);
	geometry_msgs::Pose predictPose(V_Configuration q);
	M_CartesianJacobian predictHessian(V_Configuration q,double delta=1e-6);

	bool guessParameters();
	void updateParameters(std::vector<double> delta);
	bool normalizeParameters();
};

}

#endif /* PLANAR_MODEL_H_ */
