#include "../include/belief_evolution.h"

BeliefEvolution::BeliefEvolution()
{
    nModel += 1; // 1 additional Model for goal state    

    ProblemDefinition();
    Eigen::VectorXd I_; 
    I_.setOnes(nModel); 
    wts = (1./nModel)*I_;

    // filter = make_shared<filters::UKF> (filters::UKF());
    filter = make_shared<filters::KalmanFilter> (filters::KalmanFilter());

    simulator = make_shared<Simulator> (Simulator());
    simulator->initialize(nModel);

    std::cout << "Created Belief Evolution Object." << std::endl;
    // dynamics[0]->printModelParams();
    // printGCs();
}

BeliefEvolution::~BeliefEvolution(){
}

void BeliefEvolution::setIntergationStepSize(double& step_size){
    for(int i=0; i<nModel; i++)
        dynamics[i]->integration_step_size = step_size;
}

void BeliefEvolution::setGoalGC(Eigen::Map<Eigen::VectorXd>& goal)
{
    //double threshold = 0.2;

    Guards gcs;
    std::vector<Guards> multiple_gcs;
    GC_tuple dummy;

    // GCs for Goal
    gcs.conditions.clear();
    
    for(int i=0; i<nState; i++){
        dummy.min_val = goal(i) - goal_threshold;
        dummy.max_val = goal(i) + goal_threshold;
        gcs.conditions.push_back(dummy);
    }

    multiple_gcs.push_back(gcs);
    set_of_gcs[nModel-1] = multiple_gcs;

    // Clearing vectors
    gcs.conditions.clear();
    multiple_gcs.clear();

    // Adding Goal dynamics
    /*** MICROWAVE ***/
    // auto dyna_ = make_shared<dynamics::RevolutePair> (dynamics::RevolutePair(nState, nInput, nOutput));
    // dyna_->rot_center = tf::Vector3 (-0.09505, -0.4440, 0.03572);
    // dyna_->rot_axis = tf::Quaternion (-0.635125, 0.106901, 0.747335, 0.163335);
    // dyna_->rot_radius = 0.2951;
    // dyna_->rot_orientation = tf::Quaternion (0.62752, 0.402127, -0.37856, 0.5488);
    // dyna_->scale = 100.;
    // dynamics.push_back(make_shared<dynamics::RevolutePair> (*dyna_));

    /*** DRAWER ***/
    // auto dyna_ = make_shared<dynamics::PrismaticPair> (dynamics::PrismaticPair(nState, nInput,nOutput));
    // dyna_->rigid_position = tf::Vector3 (0.016141, 0.290796, -0.117820);
    // dyna_->rigid_orientation = tf::Quaternion (0.233798, 0.972024, -0.017893, 0.013690);
    // dyna_->prismatic_dir = tf::Vector3(-0.483141, -0.875487, -0.009860);

    // dyna_->scale = 100.;

    // dynamics.push_back(make_shared<dynamics::PrismaticPair> (*dyna_));
    // dyna_.reset();

    /*** STAPLER DYNAMICS ***/
    auto dyna_1 = make_shared<dynamics::StaplerDynamicsA> (dynamics::StaplerDynamicsA(1, 1, 1, 3, 3, 3));

    // tf::Quaternion rot_offset = tf::Quaternion(0., 0.,-0.38, 0.925);
    tf::Quaternion rot_offset = tf::Quaternion(0., 0., 0., 1.);
    
    dyna_1->revolute->rot_radius = 0.058711;
    dyna_1->revolute->rot_center = tf::Vector3 (0.01095, 0.03288, 0.04342);
    dyna_1->revolute->rot_axis = tf::Quaternion (-0.13326, -0.64536, 0.295487, 0.691688)*rot_offset;    
    dyna_1->revolute->rot_orientation = tf::Quaternion (-0.65231, 0.016179, 0.726686, -0.21482);

    // dyna_1->set_planning_scale(scale);
    dyna_1->set_planning_scale(100.);

    dynamics.push_back(make_shared<dynamics::StaplerDynamicsA> (*dyna_1));
    dyna_1.reset();

    std::cout << "Goal Guard Conditions Set!" << std::endl;
}

void BeliefEvolution::getMatrices(int& idx, Eigen::Map<Eigen::VectorXd>& mu, Eigen::Map<Eigen::VectorXd>& u, Eigen::Map<Eigen::MatrixXd>& A, Eigen::Map<Eigen::MatrixXd>& B, Eigen::Map<Eigen::MatrixXd>& C, Eigen::Map<Eigen::MatrixXd>& V, Eigen::Map<Eigen::MatrixXd>& W)
{
    double delta = 1e-6*scale;
    A = dynamics[idx]->A_mat(mu, u, delta).transpose();
    B = dynamics[idx]->B_mat(mu, u, delta).transpose();
    C = dynamics[idx]->C_mat(mu, delta).transpose();
    V = dynamics[idx]->V_mat().transpose();
    W = dynamics[idx]->W_mat().transpose();

    // std::cout << "B Matrix in belief Evolution : " << dynamics[idx]->B_mat(mu, u, delta) << std::endl;

    // std::cout << "\nSet B Matrix : " << B << std::endl;
}


void BeliefEvolution::forwardKinematicsMapped(int& idx, Eigen::Map<Eigen::VectorXd>& q, Eigen::Map<Eigen::VectorXd>& pose)
{   
    geometry_msgs::Pose pos_ = dynamics[idx]->forwardKinematics(q);
    pose = dynamics::poseToVector(pos_);
}


void BeliefEvolution::inverseKinematicsMapped(int& idx, Eigen::Map<Eigen::VectorXd>& pose, Eigen::Map<Eigen::VectorXd>& q)
{
    geometry_msgs::Pose pos_ = dynamics::vectorToPose(pose);
    q = dynamics[idx]->inverseKinematics(pos_);
}


/* Belief Evolution Functions*/
void BeliefEvolution::fastWts(Eigen::VectorXd& mu, Eigen::MatrixXd& cov, Eigen::VectorXd& wts_new)
{   
    int n_pts_= nSamples;
    Eigen::EigenMultivariateNormal<double> normX_cholesk(mu, cov);
    wts_new = Eigen::VectorXd::Zero(nModel);
    
    // Generating Random Samples
    Eigen::MatrixXd pts_ = normX_cholesk.samples(n_pts_); // nState X nPts

    int idx;
    std::vector<int> idxs;
    Eigen::Map<Eigen::VectorXd> pt(pts_.col(0).data(), nState, 1);

    // #pragma omp parallel num_threads(4)
    for(int i=0; i<pts_.cols(); i++)
    {
        pt = Eigen::Map<Eigen::VectorXd>(pts_.col(i).data(), nState, 1);
        utils::activeModel(pt, idx, nState, nModel, set_of_gcs);
        idxs.push_back(idx);
    }

    std::map<int, unsigned int> wts_map = utils::counter(idxs);

    for(int i=0; i<nModel; i++)
        wts_new(i) = wts_map[i]; 

    //Normalize wts
    wts_new /= wts_new.sum();
}


/* Belief Evolution Functions*/
void BeliefEvolution::fastWtsMapped(Eigen::Map<Eigen::VectorXd>& mu, Eigen::Map<Eigen::MatrixXd>& cov, Eigen::Map<Eigen::VectorXd>& wts_new)
{   
    Eigen::VectorXd mu1;
    Eigen::MatrixXd cov1;
    Eigen::VectorXd wts1;

    mu1 = mu;
    cov1 = cov;
    
    fastWts(mu1, cov1, wts1);
    wts_new = wts1;
}


void BeliefEvolution::beliefUpdatePlanning(Eigen::Map<Eigen::VectorXd>& mu, Eigen::Map<Eigen::MatrixXd>& cov, Eigen::Map<Eigen::VectorXd>& u, Eigen::Map<Eigen::VectorXd>& mu_new, Eigen::Map<Eigen::MatrixXd>& cov_new, Eigen::Map<Eigen::VectorXd>& wts_new, int ds_new)
{
    // If we don't get as input a PD matrix, there is no need 
    // to propagate the dynamics. Hence, we just return back 
    // the erroreneous values. This can happen during optimization

    // Compute the Cholesky decomposition of cov. 
    Eigen::LLT<Eigen::MatrixXd> lltOfCov(cov); 

    if(lltOfCov.info() == Eigen::NumericalIssue)
    {   
        mu_new = mu;
        cov_new = cov; // Converting covariance to near PD matrix
        wts_new = wts;

        Eigen::MatrixXf::Index max_index;
        wts_new.maxCoeff(&max_index);
        ds_new = max_index;
        return;
    } 

    else
    {
        if(mu.hasNaN()){
            cout << "Invalid Inputs:\nmu\n " << mu << "\nCov =\n" << cov << endl;
            throw std::runtime_error("mu has Nans");
        }

        // Initialization
        Eigen::VectorXd mu_bar(nState);
        Eigen::MatrixXd cov_bar(nState, nState), dummy(nState, nState);

        // ds Prediction
        Eigen::MatrixXd P_dnew_dk = eps1*Eigen::MatrixXd::Identity(nModel, nModel);
        Eigen::VectorXd old_wts = wts;
        Eigen::VectorXd wts1, wts_bar;

        // ds Observation update
        Eigen::VectorXd P_z_ds = Eigen::VectorXd::Zero(nModel);
        Eigen::VectorXd new_wts = Eigen::VectorXd::Zero(nModel);
        Eigen::MatrixXd obs_err_cov;

        // Belief Update: Continuous Dynamics
        for(int i=0; i<nModel; i++){  
            // Filter Prediction
            filter->prediction(mu, cov, u, mu_bar, cov_bar);

            if(mu_bar.hasNaN()){
                cout << "Error in mu_bar" << mu_bar.transpose() << "\nCov =\n" << cov << endl;
                throw std::runtime_error("mu_bar has Nans");
            }
            
            // MLO Update 
            filter->update(mu_bar, cov_bar, mu_bar, mu_set_[i], cov_set_[i]);
            
            if(mu_set_[i].hasNaN()){
                cout << "Error in mu_set for model " << i << endl;
                cout << "Mu: "<< mu_set_[i].transpose() << "\nCov =\n" << cov_set_[i] << endl;
                throw std::runtime_error("mu_set_ has Nans");
            }

            // Discrete State Update: Generating 
            // Extended State Transition Matrix
            fastWts(mu_set_[i], cov_set_[i], wts1);
            P_dnew_dk.row(i) = wts1;
        }

        // Discrete State Belief: Prior
        wts_bar = P_dnew_dk.transpose()*old_wts;

        if(wts_bar.hasNaN() || wts_bar.maxCoeff() < 1e-7){
            cout << "wts in dyna = \n" << wts << endl;
            cout << "Check wts_bar = \n" << wts_bar << endl;
            cout << "Setting wts_bar at = \n" << old_wts << endl;
            cout << "P_dnew_dk = \n" << P_dnew_dk << endl;
            wts_bar = old_wts;
            throw std::runtime_error( "Error encountered in ds_wts calculation!" );
        }
        else
            wts_bar /= wts_bar.sum();

        // Update Continous State Belief Mean Prior 
        utils::multiply_vectorMap_w_vector(mu_set_, wts_bar, mu_new);

        // cout << "Prior Values in Belief Update: " << endl;
        // cout << "mu : " << mu_new.transpose() << endl;

        //* Discrete Stae Update: Posterior *//
        try{
            for(int i=0; i<nModel; i++) {   
                utils::nearestPD(cov_set_[i], cov_set_[i]); // Ensuring a PD covariance
            
                // Construct Observation Probability Matrix
                P_z_ds(i) = utils::mvn_pdf(mu_new, mu_set_[i], cov_set_[i]);
            }

            if(P_z_ds.hasNaN() || P_z_ds.maxCoeff() < 1e-8)
                new_wts = wts_bar; // Just Use Prior
            else{
                P_z_ds /= P_z_ds.sum();
                new_wts = P_z_ds.array() * wts_bar.array();

                if(!new_wts.allFinite() || new_wts.isZero())
                    new_wts = wts_bar;
                else
                    new_wts /= new_wts.sum();
                
                // cout << "P_z_ds = "<< P_z_ds.transpose()<< endl;
                // cout << "wts_bar = " << wts_bar.transpose() << endl;
                // cout << "new_wts = " << new_wts.transpose() << endl; 
                // cout << "sum of new_wts = " << new_wts.sum() << endl;
            }
        } 

        catch(const char* msg)
        { 
            cerr << msg << endl;
        }

        wts = new_wts;
        utils::multiply_vectorMap_w_vector(mu_set_, new_wts, mu_new);
        
        // Additional Uncertainty due to difference in estimation 
        // of mu using multi-modal and individual model
        for(int i=0; i<nModel; i++){   
            dummy = cov_set_[i] + (mu_set_[i]-mu_new)*(mu_set_[i]-mu_new).transpose();
            utils::nearestPD(dummy, dummy);
            cov_set_[i] = dummy;
        }
        utils::multiply_matrixMap_w_vector(cov_set_, new_wts, cov_new);
        Eigen::MatrixXf::Index max_index;
        wts.maxCoeff(&max_index);

        ds_new = max_index;
        activeIdx = ds_new;
        wts_new = wts;

        // cout << "\nPosterior Values in Belief Update: " << endl;
        // cout << "mu_new : " << mu_new.transpose() << endl;
    }
}


void BeliefEvolution::predictionStochastic(Eigen::Map<Eigen::VectorXd>& mu, Eigen::Map<Eigen::MatrixXd>& cov, Eigen::Map<Eigen::VectorXd>& u, Eigen::Map<Eigen::VectorXd>& mu_new, Eigen::Map<Eigen::MatrixXd>& cov_new, Eigen::Map<Eigen::VectorXd>& wts_new, int& ds_new)
{
    /*Function to do Belief Prediction by mixing mulitple models
     */
    Eigen::MatrixXd P_dnew_dk;

    // Discrete Dynamics
    P_dnew_dk = eps1*Eigen::MatrixXd::Identity(nModel, nModel);
    Eigen::VectorXd old_wts = wts;
    Eigen::VectorXd wts_bar;

    Eigen::VectorXd wts1;

    // increaseIntegrationResolution(1., ds_res_loop_count_);
    
    // Continuous Dynamics
    for(int l_count=0; l_count<ds_res_loop_count_; l_count++){
        for(int i=0; i<nModel; i++){   
            //dynamics[i]->propagateStateWithCov(mu, cov, u, mu_set_[i], cov_set_[i]);
            filter->prediction(mu, cov, u, mu_set_[i], cov_set_[i]);

            // Discrete Dynamics
            fastWts(mu_set_[i], cov_set_[i], wts1);
            P_dnew_dk.row(i) = wts1;
        }

        wts_bar = P_dnew_dk.transpose()*old_wts;

        if(wts_bar.hasNaN())
            wts_bar = old_wts;
        else
            wts_bar /= wts_bar.sum();

        wts = wts_bar;
        utils::multiply_vectorMap_w_vector(mu_set_, wts_bar, mu_new);
        utils::multiply_matrixMap_w_vector(cov_set_, wts_bar, cov_new);
        mu = mu_new;
        cov = cov_new;
    }

    // Data Storage
    Eigen::MatrixXf::Index max_index;
    wts.maxCoeff(&max_index);
    ds_new = max_index;
    activeIdx = ds_new;
    wts_new = wts;

    /*// Verbose
    std::cout << "Values after Prediction Step" << std::endl;
    std::cout << "mu = " << mu.transpose() << std::endl;
    std::cout << "cov =" << cov << std::endl;
    */
}


void BeliefEvolution::predictionDeterministic(Eigen::Map<Eigen::VectorXd>& mu, Eigen::Map<Eigen::MatrixXd>& cov, Eigen::Map<Eigen::VectorXd>& u, Eigen::Map<Eigen::VectorXd>& mu_new, Eigen::Map<Eigen::MatrixXd>& cov_new, Eigen::Map<Eigen::VectorXd>& wts_new, int ds_new)
{
    /* Function to do belief prediction based on the
     * Most Likley Discrete state Model
     * */

    utils::activeModel(mu, activeIdx, nState, nModel, set_of_gcs);
    dynamics[activeIdx]->propagateStateWithCov(mu, cov, u, mu_set_[activeIdx], cov_set_[activeIdx]);

    // Reset weights
    wts = Eigen::VectorXd::Zero(nModel);
    wts(activeIdx) = 1.0;

    utils::multiply_vectorMap_w_vector(mu_set_, wts, mu_new);
    utils::multiply_matrixMap_w_vector(cov_set_, wts, cov_new);

    Eigen::MatrixXf::Index max_index;
    wts.maxCoeff(&max_index);

    // Data Storage
    ds_new = max_index;
    activeIdx = ds_new;
    wts_new = wts;
}


void BeliefEvolution::observationUpdate(Eigen::Map<Eigen::VectorXd>& z, Eigen::Map<Eigen::VectorXd>& mu_new, Eigen::Map<Eigen::MatrixXd>& cov_new, Eigen::Map<Eigen::VectorXd>& wts_new, int ds_new)
{
    // Discrete State Update
    Eigen::VectorXd wts_bar = wts; //  Previous wts
    Eigen::VectorXd P_z_ds = Eigen::VectorXd::Zero(nModel);
    Eigen::VectorXd new_wts = Eigen::VectorXd::Zero(nModel);
    Eigen::MatrixXd obs_err_cov;

    // Continuous State update
    for(int i=0; i<nModel; i++) {   
        filter->update(mu_set_[i], cov_set_[i], z, mu_set_[i], cov_set_[i]);

        // Discrete State
        utils::nearestPD(cov_set_[i], obs_err_cov); // converting obs_err_cov mat to PD
        P_z_ds(i) = utils::mvn_pdf(z, mu_set_[i], obs_err_cov);
    }

    if(P_z_ds.hasNaN() || P_z_ds.maxCoeff() < 1e-6){
        new_wts = wts_bar;
        cout << "Something wrong with P_z_ds or" <<  
            "very low values in P_z_ds, Invoking If Condition" << endl;
    }
    else{
        P_z_ds /= P_z_ds.sum();
        new_wts = P_z_ds.array() * wts_bar.array();

        if(new_wts.hasNaN() || new_wts.sum()<1e-10)
            new_wts = wts_bar;
        else
            new_wts /= new_wts.sum();
    }

    wts = new_wts;

    utils::multiply_vectorMap_w_vector(mu_set_, new_wts, mu_new);

    // Update for proper multi-Model mixing
    for(int i=0; i<nModel; i++){
        cov_set_[i] += (mu_set_[i]-mu_new)*(mu_set_[i]-mu_new).transpose();
    }
    utils::multiply_matrixMap_w_vector(cov_set_, new_wts, cov_new);

    Eigen::MatrixXf::Index max_index;
    wts.maxCoeff(&max_index);

    ds_new = max_index;
    activeIdx = ds_new;
    wts_new = wts;

    /*
    // Verbose
    std::cout << "\nValues after Update Step" << std::endl;
    std::cout << "mu = " << mu.transpose() << std::endl;
    std::cout << "cov =" << cov << std::endl;
    cout << "wts New = " << wts.transpose() << endl;
    */
}


/* Utilities */
bool BeliefEvolution::isPDMapped(Eigen::Map<Eigen::MatrixXd>& A)
{
    Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of cov
    if(lltOfA.info() == Eigen::NumericalIssue)
        return false;
    else
        return true;
}


void BeliefEvolution::nearestPDMapped(Eigen::Map<Eigen::MatrixXd>& A, Eigen::Map<Eigen::MatrixXd>& A_hat)
{
    Eigen::MatrixXd A1;
    Eigen::MatrixXd A_hat1;

    A1 = A;   
    utils::nearestPD(A1, A_hat1);
    A_hat = A_hat1;
}


/* Functions to interact with Simualtor */
void BeliefEvolution::simulateOneStep(Eigen::Map<Eigen::VectorXd>& x, Eigen::Map<Eigen::VectorXd>& u, Eigen::Map<Eigen::VectorXd>& x_new, Eigen::Map<Eigen::VectorXd>& z_new)
{
   simulator->simulateOneStep(x, u, x_new, z_new);
}

/// Utility function
void BeliefEvolution::printGCs()
{
    std::cout << "Listing Guard Conditions for the System: " << std::endl;
    for(int i=0; i< nModel; i++){
        std::cout << "For Model " << i << " :" << std::endl;
        std::vector<Guards> gcs = set_of_gcs[i];

        for(unsigned int j=0; j<gcs.size(); j++){   
            Guards gc = gcs.at(j);
            std::cout << "GC " << j+1 << ": ";

            for(int k=0; k<nState; k++)
                std::cout << " [ "<< gc.conditions.at(k).min_val << " , " << gc.conditions.at(k).max_val << " ] ,\t" <<  std::endl;
        }

        std::cout << "\n\n" ; 
    }

}

void BeliefEvolution::setReferenceFrameMapped(Eigen::Map<Eigen::VectorXd>& trans)
{
    tf::Transform t_ = tf::Transform(tf::Quaternion(trans[3], trans[4], trans[5], trans[6]), tf::Vector3(trans[0], trans[1], trans[2]));

    filter->s_to_LA.setOrigin(t_.getOrigin());
    filter->s_to_LA.setRotation(t_.getRotation());

    // for(int i=0; i<nModel; i++)
    // {
    //     // dynamics[i]->ref_frame.setOrigin(t_.getOrigin());
    //     // dynamics[i]->ref_frame.setRotation(t_.getRotation());
    //     // dynamics[i]->setRefFrame(t_);
    // }
}

/* 
// FOllowing function can be used to increase the resolution of time step for a discrete time system. Assumptions: System is controllable and observable. Zero-order hold on the control input in the changed resolution till the original time-step. No Process noise. Observation error is random white noise
void BeliefEvolution::increaseIntegrationResolution(float k1, float k2)
{
    float ratio = k1/k2;
    
    for(int i=0; i<nModel; i++)
    {
        dynamics[i]->A = dynamics[i]->A.pow(ratio);
        dynamics[i]->B = ratio*dynamics[i]->B;
    }
}
*/


// int main()
// {
//     BeliefEvolution bel;

//     Eigen::VectorXd x(4), u(3), x_new(4);
//     x << 15., 0., 0., 0.;
//     u << 1., 0., 0.;

//     std::cout << "Testing State Propagation Functions :\n" << std::endl;
//     for(int i=0; i<4; i++)
//         bel.dynamics[i]->propagateState(x, u, x_new);

//     for(int idx=0; idx<4; idx++)
//     {
//         std::cout << "\n\n#######################\n System Matrices for Dynamics: " << idx << std::endl;
//         std::cout << "A_mat : \n" << bel.dynamics[idx]->A_mat(x, u) << std::endl;
//         std::cout << "B_mat : \n" << bel.dynamics[idx]->B_mat(x, u) << std::endl;
//         std::cout << "C_mat : \n" << bel.dynamics[idx]->C_mat(x) << std::endl;
//         std::cout << "V_mat : \n" << bel.dynamics[idx]->V_mat() << std::endl;
//         std::cout << "W_mat : \n" << bel.dynamics[idx]->W_mat() << std::endl;
//     }
// }
