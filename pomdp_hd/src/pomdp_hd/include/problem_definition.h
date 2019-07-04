#ifndef PROBLEM_DEFINITION_H_
#define PROBLEM_DEFINITION_H_

#include <iostream>
#include <iomanip>   
#include <eigen3/Eigen/Dense>
//#include <eigen3/unsupported/Eigen/MatrixFunctions> // For changing resolution of ds dynamics
#include <map>
#include <random>
#include <algorithm>
#include <cstdlib>
#include "eigenmvn.h"
#include "omp.h"
#include <cmath>
#include "utils.h"

#include "dynamics.h"
#include "dynamics_models/linear_dynamics.h"
#include "../include/dynamics_models/revolute_pair.h"
#include "../include/dynamics_models/prismatic_pair.h"
#include "../include/dynamics_models/stapler_dynamics.h"
#include "../src/dynamics_models/stapler/stapler_dynamics_A.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_B.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_C.cpp"
#include "../src/dynamics_models/stapler/stapler_dynamics_D.cpp"


#include <memory>

using namespace std;

class ProblemDefinition
{
    public:
		int nState, nInput, nOutput, nModel;

        // Scaling up dimensions for planning
        double scale; 

        std::vector<std::shared_ptr<dynamics::Dynamics> > dynamics;

        std::map<int, std::vector<Guards> > set_of_gcs;
       
        ProblemDefinition(); 
        void defineHybridDynamicsModel();
        void defineGuardConditions();
};

#endif // PROBLEM_DEFINITION_H_

