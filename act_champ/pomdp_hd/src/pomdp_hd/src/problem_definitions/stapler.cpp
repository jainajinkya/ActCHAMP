#include "../../include/problem_definition.h"

ProblemDefinition::ProblemDefinition()
{
    nState = 4;
    nInput = 3;
    nOutput = 4;
    nModel = 4;
    scale = 100.;
    defineHybridDynamicsModel();
    defineGuardConditions();
}

/* Dynamics Functions*/
void ProblemDefinition::defineHybridDynamicsModel()
{
    auto rev = make_shared<dynamics::RevolutePair> (dynamics::RevolutePair(1, 1, 1));

    // tf::Quaternion rot_offset = tf::Quaternion(0., 0.,-0.38, 0.925);
    tf::Quaternion rot_offset = tf::Quaternion(0., 0., 0., 1.);

    rev->rot_radius = 0.058711;
    rev->rot_center = tf::Vector3 (0.01095, 0.03288, 0.04342);
    rev->rot_axis = tf::Quaternion (-0.13326, -0.64536, 0.295487, 0.691688)*rot_offset;
    rev->rot_orientation = tf::Quaternion (-0.65231, 0.016179, 0.726686, -0.21482);
    
    rev->scale = scale;


    // /* Revolute No Contact*/
    auto dyna_1 = make_shared<dynamics::StaplerDynamicsC> (dynamics::StaplerDynamicsC(1, 1, 1, 3, 3, 3));
    dyna_1->revolute = rev;
    dyna_1->set_planning_scale(scale);
    dynamics.push_back(make_shared<dynamics::StaplerDynamicsC> (*dyna_1));
    dyna_1.reset();


    // /* Revolute Line Contact */
    auto dyna_2 = make_shared<dynamics::StaplerDynamicsB> (dynamics::StaplerDynamicsB(1, 1, 1, 3, 3, 3));
    dyna_2->revolute = rev;
    dyna_2->set_planning_scale(scale);
    dynamics.push_back(make_shared<dynamics::StaplerDynamicsB> (*dyna_2));

    dyna_2.reset();


    // /* Revolute Surface Contact */
    auto dyna_3 = make_shared<dynamics::StaplerDynamicsD> (dynamics::StaplerDynamicsD(1, 1, 1, 3, 3, 3));
    dyna_3->revolute = rev;
    dyna_3->set_planning_scale(scale);
    dynamics.push_back(make_shared<dynamics::StaplerDynamicsD> (*dyna_3));

    dyna_3.reset();


    /* Rigid Model */
    auto dyna_4 = make_shared<dynamics::StaplerDynamicsA> (dynamics::StaplerDynamicsA(1, 1, 1, 3, 3, 3));

    dyna_4->revolute = rev;
    dyna_4->set_planning_scale(scale);
    dynamics.push_back(make_shared<dynamics::StaplerDynamicsA> (*dyna_4));
    dyna_4.reset();

}



void ProblemDefinition::defineGuardConditions()
{
   double INF = 1e3*scale;

    Guards gcs;
    std::vector<Guards> multiple_gcs;
    GC_tuple dummy;

    double theta_max = -1.12*scale; // 1.57*scale;
    double theta_min = -2.22*scale;
    double theta_flat = -1.75*scale; // 0.90*scale; // 0.51
    // double z_touch =  0.105*scale; ////0.072*scale;  // 0.105
    double z_touch =  -0.10*scale; 
    double z_table = -0.14*scale; //-0.05*scale;


    ///* GCs for model 1 :  Revolute No Contact */
    // State 0: x
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 1: y
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 2: z
    dummy.min_val = z_touch;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 3 : \theta
    // dummy.min_val = theta_max;
    // dummy.max_val = INF;
    dummy.min_val = theta_min;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);

    multiple_gcs.push_back(gcs);
    set_of_gcs[0] = multiple_gcs;

    gcs.conditions.clear();
    multiple_gcs.clear();


    ///* GCs for model 2 :  Revolute; Line Contact */
    // State 0: x
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 1: y
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 2: z
    dummy.min_val = z_table;
    dummy.max_val = z_touch;
    gcs.conditions.push_back(dummy);
    // State 3 : \theta
    // dummy.min_val = theta_flat;
    // dummy.max_val = theta_max;
    dummy.min_val = theta_min;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);

    multiple_gcs.push_back(gcs);
    set_of_gcs[1] = multiple_gcs;

    gcs.conditions.clear();
    multiple_gcs.clear();


    ///* GCs for model 3 :  Revolute Surface Contact */
    // State 0: x
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 1: y
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 2: z
    dummy.min_val = -INF;
    dummy.max_val = z_table;
    gcs.conditions.push_back(dummy);
    // State 3 : \theta
    // dummy.min_val = theta_min;
    // dummy.max_val = theta_flat;
    dummy.min_val = theta_min;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);

    multiple_gcs.push_back(gcs);
    set_of_gcs[2] = multiple_gcs;

    gcs.conditions.clear();
    multiple_gcs.clear();


    ///* GCs for model 4 :  Rigid */
    // State 0: x
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 1: y
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 2: z
    dummy.min_val = -INF;
    dummy.max_val = INF;
    gcs.conditions.push_back(dummy);
    // State 3 : \theta
    dummy.min_val = -INF;
    dummy.max_val = theta_min;
    gcs.conditions.push_back(dummy);

    multiple_gcs.push_back(gcs);
    set_of_gcs[3] = multiple_gcs;

    gcs.conditions.clear();
    multiple_gcs.clear();


    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // /* Old Definition */
    // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    // /*  Discrete State 0  */
    // // State 0: x 
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -0.07*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC 2
    // // State 0: x
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.78*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC3
    // dummy.min_val = 0.95*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -0.22*scale;
    // dummy.max_val = 0.20*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC4
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = -0.22*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC5
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = 0.20*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);

    // set_of_gcs[0] = multiple_gcs;

    // gcs.conditions.clear();
    // multiple_gcs.clear();


    // /*  Discrete State 1 */
    // // GCs for model 1: Rigid on Table
    // // State 0: x
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = 0.95*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -0.22*scale;
    // dummy.max_val = 0.20*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // // dummy.max_val = -0.07*scale;
    // dummy.max_val = 0.10*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.0*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // set_of_gcs[1] = multiple_gcs;

    // gcs.conditions.clear();
    // multiple_gcs.clear();


    // /*  Discrete State 2 */
    // // GCs for model 2: Revolute not on Table
    // // State 0: x
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // // dummy.min_val = 0.15*scale;
    // dummy.min_val = 0.10*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();


    // // GC 2
    // gcs.conditions.clear();
    // // State 0: x
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.78*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();


    // // GC3
    // dummy.min_val = 0.95*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -0.22*scale;
    // dummy.max_val = 0.20*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC4
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -INF*scale;
    // dummy.max_val = -0.22*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // gcs.conditions.clear();

    // // GC5
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = 0.20*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // dummy.min_val = -INF*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);


    // set_of_gcs[2] = multiple_gcs;

    // gcs.conditions.clear();
    // multiple_gcs.clear();


    // // GCs for model 3: Revolute on Table
    // // State 0: x
    // dummy.min_val = 0.78*scale;
    // dummy.max_val = 0.95*scale;
    // gcs.conditions.push_back(dummy);
    // // State 1: y
    // dummy.min_val = -0.22*scale;
    // dummy.max_val = 0.20*scale;
    // gcs.conditions.push_back(dummy);
    // // State 2: z
    // // dummy.min_val = -0.07*scale;
    // // dummy.max_val = 0.15*scale;
    // dummy.min_val = -INF*scale;
    // dummy.max_val = 0.10*scale;
    // gcs.conditions.push_back(dummy);
    // // State 3 : \theta
    // dummy.min_val = 0.0*scale;
    // dummy.max_val = INF*scale;
    // gcs.conditions.push_back(dummy);

    // multiple_gcs.push_back(gcs);
    // set_of_gcs[3] = multiple_gcs;

    // gcs.conditions.clear();
    // multiple_gcs.clear();
}
