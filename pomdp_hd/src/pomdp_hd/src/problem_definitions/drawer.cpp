#include "../../include/problem_definition.h"

ProblemDefinition::ProblemDefinition()
{
    nState = 1;
    nInput = 1;
    nOutput = 1;
    nModel = 1;
    defineHybridDynamicsModel();
    defineGuardConditions();
}

/* Dynamics Functions*/
void ProblemDefinition::defineHybridDynamicsModel()
{
    // Dynamics 1
    auto dyna_ = make_shared<dynamics::PrismaticPair> (dynamics::PrismaticPair(nState, nInput,nOutput));
    
    // dyna_->rigid_position = tf::Vector3 (0.091469, -0.3459, 0.2237);
    // dyna_->rigid_orientation = tf::Quaternion (-0.00058, 0.003192, -0.03214, 0.9995);
    // dyna_->prismatic_dir = tf::Vector3(-0.00639, -0.02686, -0.9996);

    dyna_->rigid_position = tf::Vector3 (0.016141, 0.290796, -0.117820);
    dyna_->rigid_orientation = tf::Quaternion (0.233798, 0.972024, -0.017893, 0.013690);
    dyna_->prismatic_dir = tf::Vector3(-0.483141, -0.875487, -0.009860);

    dyna_->scale = 100.;

    dynamics.push_back(make_shared<dynamics::PrismaticPair> (*dyna_));
    dyna_.reset();
}


void ProblemDefinition::defineGuardConditions()
{
   double INF = 10000.;

    // GCs for model 0
    Guards gcs;
    std::vector<Guards> multiple_gcs;
    GC_tuple dummy;

    // State 0
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);

    multiple_gcs.push_back(gcs);
    set_of_gcs[0] = multiple_gcs;

    // Clearing vectors
    gcs.conditions.clear();
    multiple_gcs.clear();
}
