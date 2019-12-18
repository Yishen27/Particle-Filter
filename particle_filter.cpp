/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <list>
#include <cmath>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   
   */
  num_particles = 50;  // TODO: Set the number of particles 
  
  
  double std_x = std[0];
  double std_y = std[1];
  double std_th = std[2];
  
  
  std::default_random_engine r; 
  std::normal_distribution<double> x_noisy(0,std_x);
  std::normal_distribution<double> y_noisy(0,std_y);
  std::normal_distribution<double> t_noisy(0,std_th);
  
  for (int i = 0; i<num_particles; ++i){
    struct Particle P;
    P.id = i;
    P.x = x+x_noisy(r);
    P.y = y+y_noisy(r);
    P.theta = theta+t_noisy(r);
    P.weight = 1;
    particles.push_back(P);
  
    
  }
  is_initialized = true;
  
  
  /**
   cout<<"id"<<P.id<<endl;
    cout<<"x"<<P.x<<endl;
    cout<<"y"<<P.y<<endl;
    cout<<"T"<<P.theta<<endl;
    cout<<"w"<<P.weight<<endl;
    */

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  
  double xt;
  double yt;
  double thetat;
  

  for(int i=0;i<particles.size(); i++){

  	if(fabs(yaw_rate)>0.001){

  		xt = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
    	yt = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
    	thetat = particles[i].theta + yaw_rate*delta_t;
    	
    }
    else{
    	xt = particles[i].x + velocity*delta_t*cos(particles[i].theta);
    	yt = particles[i].y + velocity*delta_t*sin(particles[i].theta);
    	thetat = particles[i].theta;

    }
    cout<<"predicted x= "<<xt<<endl;
    cout<<"predicted y= "<<yt<<endl;
    cout<<"predicted tt= "<<thetat<<endl;
  
    std::default_random_engine e; 
    std::normal_distribution<double> x_noise(0,std_pos[0]);
    std::normal_distribution<double> y_noise(0,std_pos[1]);
    std::normal_distribution<double> t_noise(0,std_pos[2]);
  
    xt += x_noise(e);
    yt += y_noise(e);
    thetat += t_noise(e);
    
    particles[i].x = xt;
    particles[i].y = yt;
    particles[i].theta = thetat;

  

  }
    
    /**
    cout<<"predicted x= "<<xt<<endl;
    cout<<"predicted y= "<<yt<<endl;
    cout<<"predicted tt= "<<thetat<<endl;
    */
  }
  


void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  cout<<"data association started"<<endl;
  for(int i=0;i<observations.size(); i++){
    double min_distance = 1000000000;
    for(int j=0; j<predicted.size(); j++){
      double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
      if (distance<min_distance){
        min_distance = distance;
        observations[i].id = predicted[j].id;        
      } 
      
    }
  
  }
  cout<<"data association ended"<<endl;

  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  
  for (int i = 0; i<particles.size(); i++){
    vector<LandmarkObs> transformed_ob;

    /** 
    transform the observations to global coordinate
    */
    for (int j = 0; j<observations.size();j++){
      struct LandmarkObs L;
      L.id = observations[i].id;
      double x_p = particles[i].x;
      double y_p = particles[i].y;
      double theta_p = particles[i].theta;
      L.x = x_p + (cos(theta_p) * observations[j].x) - (sin(theta_p) * observations[j].y);
      L.y = y_p + (sin(theta_p) * observations[j].x) - (sin(theta_p) * observations[j].y);
    
    transformed_ob.push_back(L);
    }
    
    /** 
    find landmarks in sensor range
    */
    std::vector<LandmarkObs> LM_in_range;
    for (int k=0;k<map_landmarks.landmark_list.size();k++){
      if (dist(map_landmarks.landmark_list[k].x_f,map_landmarks.landmark_list[k].y_f,particles[i].x,particles[i].y)<sensor_range){
        struct LandmarkObs L_M;
        L_M.x = map_landmarks.landmark_list[k].x_f;
        L_M.y = map_landmarks.landmark_list[k].y_f;
        L_M.id = map_landmarks.landmark_list[k].id_i;
        LM_in_range.push_back(L_M);
      }
    }
    cout<<"vec LM built"<<endl;
    dataAssociation(LM_in_range,transformed_ob);
    
    /** 
    calculate weight
    */
    for (int l = 0; l<LM_in_range.size();l++){
      double mu_x = LM_in_range[l].x;
      double mu_y = LM_in_range[l].y;
      cout<<"mu_x ="<<mu_x<<endl;
      cout<<"mu_y ="<<mu_y<<endl;
      double obs_x;
      double obs_y;
      for (int m=0;m<transformed_ob.size();m++){
        if (transformed_ob[m].id == LM_in_range[l].id){
          obs_x = transformed_ob[m].x;
          obs_y = transformed_ob[m].y;
        }
      }
      cout<<"obs_x ="<<obs_x<<endl;
      cout<<"obs_y ="<<obs_y<<endl;
      
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      cout<<"sig_x ="<<sig_x<<endl;
      cout<<"sig_y ="<<sig_y<<endl;
      
      double gauss_norm;
      gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);
      cout<<"gauss_norm ="<<gauss_norm<<endl;
      double exponent;
      exponent = (pow(obs_x- mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(obs_y - mu_y, 2) / (2 * pow(sig_y, 2)));
      cout<<"exponent ="<<exponent<<endl;
      double weight;
      weight = gauss_norm * exp(-exponent);
      cout<<"calculated weight "<<weight<<endl;
      
      double previous_w = particles[i].weight;
      cout<<"previous weight of particle "<<i<<" is "<<previous_w<<endl;
      
      
      double update = previous_w*weight;
      particles[i].weight = update; 
      
    }
      
  }
     
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  /**
  cout<<"resample started"<<endl;
  */
  vector<Particle> particles_r;
  
  int N = particles.size();
  vector<int> w;
  for (int i = 0; i<N; i++){
    w.push_back(particles[i].weight);
  }
  /**std::random_device re;
  std::discrete_distribution<> w;
  
  std::vector<Particle> particles_r;
  */
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, N);
  int index = dis(gen);

  double mw = *max_element(w.begin(), w.end());
  double beta = 0;
  
  for (int j=0;j<N;j++){
    beta += dis(gen)*2*mw;
    while (beta>w[index]){
      beta-=w[index];
      index = (index+1)%N;
      
    }
        
    particles_r.push_back(particles[index]);
    
  }
  
  particles = particles_r;
  

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}