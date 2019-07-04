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
    auto dyna_ = make_shared<dynamics::RevolutePair> (dynamics::RevolutePair(nState, nInput,nOutput));
    
    // dyna_->rot_center = tf::Vector3 (-0.19878, -0.12125, -0.04665);
    // dyna_->rot_axis = tf::Quaternion (0.68284, -0.19925, 0.673344, 0.20157);
    // dyna_->rot_radius = 0.3435;
    // dyna_->rot_orientation = tf::Quaternion (-0.62158, -0.32967, -0.32836, 0.63018);

    // dyna_->rot_center = tf::Vector3 (-0.217038, -0.112152, -0.047928);
    // dyna_->rot_axis = tf::Quaternion (-0.29286, -0.639938, -0.14849, -0.694740);
    // dyna_->rot_radius = 0.343744;
    // dyna_->rot_orientation = tf::Quaternion (0.612293, 0.181621, -0.354693, 0.682865);

    dyna_->rot_center = tf::Vector3 (-0.09505, -0.4440, 0.03572);
    dyna_->rot_axis = tf::Quaternion (-0.635125, 0.106901, 0.747335, 0.163335);
    dyna_->rot_radius = 0.2951;
    dyna_->rot_orientation = tf::Quaternion (0.62752, 0.402127, -0.37856, 0.5488);

    dyna_->scale = 100.;

    dynamics.push_back(make_shared<dynamics::RevolutePair> (*dyna_));
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
