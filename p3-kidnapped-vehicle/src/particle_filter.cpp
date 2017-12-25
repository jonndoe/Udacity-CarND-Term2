/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// declare a random number engine
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std_pos[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 110;

  // define normal (Gaussian) distributions for sensor noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  // initialize particles
  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;
    p.weight = 1.0;

    // add noise
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);

    particles.push_back(p);
		weights.push_back(p.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// define normal (Gaussian) distributions for sensor noise
	normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

	// add measurements to each particle
	  for (int i = 0; i < num_particles; i++) {

	    // calculate new state
	    if (fabs(yaw_rate) < 0.00001) {
	      particles[i].x += velocity * delta_t * cos(particles[i].theta);
	      particles[i].y += velocity * delta_t * sin(particles[i].theta);
	    }
	    else {
	      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
	      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
	      particles[i].theta += yaw_rate * delta_t;
	    }

	    // add noise
	    particles[i].x += dist_x(gen);
	    particles[i].y += dist_y(gen);
	    particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	// get the closest predicted measurement to each observed measurement
	for (int i = 0; i < observations.size(); i++) {

		// current observation
		LandmarkObs observation = observations[i];

		// init minimum distance to maximum possible
    double min_distance = numeric_limits<double>::max();

		// init id of landmark to associate with closest measurement
		int landmark_id = -1;

		for(int j=0; j<predicted.size(); j++) {

					// get current prediction
					LandmarkObs predicted_measurement = predicted[j];

					// calculate distance between current vs. predicted landmarks
        	double distance = dist(observation.x, observation.y, predicted_measurement.x, predicted_measurement.y);

					// find the closest landmark
					if (distance < min_distance) {
            	min_distance = distance;
            	landmark_id = predicted_measurement.id;
        	}
        }
        // assign the observed measurement to specific landmark
        observations[i].id = landmark_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// iterate through each particle
	for (int i = 0; i < num_particles; i++) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// keep predicted landmark locations within sensor range of particle
    	vector<LandmarkObs> predicted_landmarks;

    	for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

				// get id and x,y coordinates
    		int lm_id = map_landmarks.landmark_list[j].id_i;
    		double lm_x = map_landmarks.landmark_list[j].x_f;
    		double lm_y = map_landmarks.landmark_list[j].y_f;
    		LandmarkObs curr_lm = {lm_id, lm_x, lm_y};

    		// select landmark within range
    		if (fabs(dist(lm_x, lm_y, x, y)) <= sensor_range){
        		predicted_landmarks.push_back(curr_lm);
      		}
    	}
    	// transform points
    	vector<LandmarkObs> transformed_observations;

    	for (int k = 0; k < observations.size(); k++) {
    		double tr_x = observations[k].x*cos(theta) - observations[k].y * sin(theta) + x;
    		double tr_y = observations[k].x*sin(theta) + observations[k].y * cos(theta) + y;

    		LandmarkObs transformed_ob;
    		transformed_ob.id = observations[k].id;
    		transformed_ob.x = tr_x;
    		transformed_ob.y = tr_y;

    		transformed_observations.push_back(transformed_ob);

    	}

    	dataAssociation(predicted_landmarks, transformed_observations);

    	double total_weight = 1.0;
    	weights[i] = 1.0;
    	vector<int> associations_vec;
    	vector<double> sense_x_vec;
    	vector<double> sense_y_vec;

    	for (int l = 0; l < transformed_observations.size(); l++) {

				// placeholders for observation and prediction coordinates
    		int obv_id = transformed_observations[l].id;
    		double obv_x = transformed_observations[l].x;
    		double obv_y = transformed_observations[l].y;
    		double pred_x;
    		double pred_y;

				// get coordinates of the prediction for the current observation
    		for (int m = 0; m < predicted_landmarks.size(); m++) {
    			if (predicted_landmarks[m].id == obv_id){
    				pred_x = predicted_landmarks[m].x;
    				pred_y = predicted_landmarks[m].y;
    			}
    		}

    		// update weight for this observation
    		double w = (1/(2*M_PI*std_landmark[0]*std_landmark[1])) *
                            exp(-(pow(pred_x-obv_x, 2)/(2*pow(std_landmark[0],2)) +
                                pow(pred_y-obv_y, 2)/(2*pow(std_landmark[1],2)) -
                                (2*(pred_x-obv_x)*(pred_y-obv_y)/(sqrt(std_landmark[0])*sqrt(std_landmark[1])))));

            total_weight *= w;
            associations_vec.push_back(obv_id);
            sense_x_vec.push_back(obv_x);
            sense_y_vec.push_back(obv_y);
        }
        particles[i].weight = total_weight;
        weights[i] = total_weight;

        SetAssociations(particles[i], associations_vec, sense_x_vec, sense_y_vec);
    	predicted_landmarks.clear();
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// use discrete distribution for weights
	discrete_distribution<int> index(weights.begin(), weights.end());

	// placeholder for resampled particles
	vector<Particle> resampled_particles;

	// resample particles
	for (int i = 0; i < num_particles; i++) {
		resampled_particles.push_back(particles[index(gen)]);
	}
	particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

		// clear previous associations
		particle.associations.clear();
		particle.sense_x.clear();
		particle.sense_y.clear();

		particle.associations = associations;
	 	particle.sense_x = sense_x;
	 	particle.sense_y = sense_y;

	 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
