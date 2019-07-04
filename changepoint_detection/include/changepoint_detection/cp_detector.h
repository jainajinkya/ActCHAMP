/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#ifndef CP_DETECTOR_H_
#define CP_DETECTOR_H_

#include "ros/ros.h"
#include "changepoint_detection/Detectchangepoint_detections.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <queue>

namespace changepoint_detection{

class ModelParams
{
public:
    ModelParams(){};
    virtual ~ModelParams(){};
    
    virtual void printParams() = 0;
    virtual void fillParams(ModelSegment &seg) = 0;
    virtual std::string getModelName() = 0;
    
    double getModelEvidence(){return modelEvidence;}
    double getLogLikelihood(){return logLikelihood;}
    
    double modelEvidence;  //If params integrated, then Bayes model evidence, otherwise probably BIC
    double logLikelihood;
};


class ModelFitter{
public:    
    ModelFitter(){mp=NULL;}
    virtual ~ModelFitter(){};
    
    virtual void initialize(int model_id) = 0;
    virtual void copyFrom(ModelFitter *rhs) = 0;
    // Fit a model to the segment of start+1 to end. Params+BIC+LL in mp should be filled by fit.
    virtual bool fitSegment(double **data, const int start, const int end) = 0;
    virtual int nModels() = 0;  // Returns number of model types for this set of models    
    // Calculates arbitrary model-specific stats for a specified data segment under the current model
    virtual std::vector< std::vector<double> > calcFinalSegStats(double **data, const int start, const int end) = 0;
    
    // Additional function for CHAMP + Action
    virtual bool fitSegmentFromAction(double **data, double **actions, const int start, const int end) = 0;    
    
    ModelParams *mp;
};


class Particle
{
public:
    Particle(){};
    Particle(double prev_MAP, int pos, ModelFitter &mf){
        this->prev_MAP = prev_MAP;
        this->pos = pos;
        MAP = -INFINITY;
        nMAP = -INFINITY;
        fitter = &mf;
    }
    ~Particle(){};
    
    double MAP;
    double nMAP;
    double prev_MAP;
    int pos;
    ModelFitter *fitter;
};


class CPDetector
{
public:
    // CPDetector(const std::vector<changepoint_detection::DataPoint> data_pts, 
    //            const changepoint_detection::CPParams cp_params);
    CPDetector(const std::vector<changepoint_detection::DataPoint> data_pts, 
               const changepoint_detection::CPParams cp_params, 
               bool actions_available = false, 
               const std::vector<changepoint_detection::DataPoint> action_set = std::vector<changepoint_detection::DataPoint>());
    ~CPDetector();
    std::vector<ModelSegment> detectchangepoint_detections(std::string class_type);
    
private:
    int d_len, d_dim;
    double **data;
    bool actions_available;
    double **actions;
    std::vector<Particle> particles;
    std::queue<double> prev_max_MAP;
    std::vector <std::string> model_names;
    std::map<std::string, double> modelLogLikelihoods;


    
    double gaussCDF(double t);
    double logLenPDF(int t);
    double logOneMinLenCDF(int t);
    void normalizeMAP();
    double calculateAlpha(int M);
    void resampleParticles(int max_particles, int resamp_particles);
    // void accumulateLogLikelihoods(std::vector<Particle> particles, int t, std::ofstream& modelLLs_file);
    void generateFinalModelEvidences(double **, const int , const int, std::ofstream&);
    void generateFinalModelEvidencesFromActions(double **, double **, const int, const int, std::ofstream&);


    
    //Params
    double LEN_MEAN;                // Mean of segment length gaussian 
    double LEN_SIG;                 // Sigma of segment length gaussian
    int MIN_SEG_LEN;                // The minimum length of a segment for model fitting purposes
    int MAX_PARTICLES;              // The most particles to ever keep in the filter
    int RESAMP_PARTICLES;           // The number of particles to resample back down to when resampling 
};

} // end namespace

#endif /* CP_DETECTOR_H_ */
