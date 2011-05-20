/** Particle filter utility. **/

#ifndef _PF_
#define _PF_

#include "common.hpp"
#include <map>

class Particle
{	
  public:
	map<int, Landmark> landmarks;
	Vector3f pos;
	float weight;
	
	Particle(Vector3f _pos): pos(_pos) {}
	
	void process_measure(vector<MeasureResult>& results_measure, Matrix3f& R)
	{
		for(int i = 0; i < results_measure.size(); i++)
		{
			/** if the corresponding landmark record is found in the list, update the weight;
			 *	if not, create a new record of this predicted landmark for the particle. **/
			if(landmarks[result_measure.at(i).id])
			{
				int id = results_measure[i].id;
				Vector3f residual;
				float distance_x = landmarks[id](0) - pos(0);
				float distance_y = landmarks[id](1) - pos(1);
				residual(0) = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
				residual(1) = atan2(distance_y, distance_x);
				residual(2) = 0; 
				
				Matrix3f& G = results_measure[i].jacobian;
				
				Matrix3f Q = G.transpose() * landmarks[id].cov * G + R;
				Matrix3f K = landmarks[id].cov * G * Q.inverse();
				
				landmarks[id].pos += K * residual;
				landmarks[id].cov = Matrix3f::Identity() - K * G.transpose() * landmarks[id].cov;
				
				weight *= (2 * pi * Q).norm() ^ (-0.5) *
						  exp(-0.5 * residual.transpose() * Q.inverse() * residual);
			}
			else
			{
				Landmark new_landmark;
				new_landmark.pos <<
					pos(0) + cos(result_measure[i].result(1)) * result_measure[i].result(0),
					pos(1) + sin(result_measure[i].result(1)) * result_measure[i].result(0),
					0;
				Matrix3f jacob_inv = result_measure[i].jacobian.inverse();
				cov = jacob_inv * R * jacob_inv.transpose();
			}
		}
	}
};

class ParticleFilter
{
  public:
	vector<Particle> particles;
	Matrix3f R;
	
	ParticleFilter() {}
	
	void setR(Matrix3f& _R)
	{
		R = _R;
	}
	
	void process_measure(vector<MeasureResult>& results_measure)
	{
		for(int i = 0; i < particles.size(); i++)
		{
			particles[i].process_measure(results_measure, R);
		}
	}
	
	/**	Keep / duplicate particles that have greater weights
	 *	while giving low-weight particles some reasonable chances to keep alive
	 *	using "roulette wheel" algorithm. **/
	void resample()
	{
		/** Normalize the weights so that their sum equals to 1, **/
		sum_weight = 0;
		for(int i = 0; i < particles.size(); i++) sum_weight += particles[i].weight;
		for(int i = 0; i < particles.size(); i++) particles[i].weight /= sum_weight;
		
		/** Play the roulette wheel. **/
		vector<Particle> new_particles;
		cur_weight = particles[0].weight;
		cur_id = 0;
		float r = uniformrnd();
		/** Spin the wheels for particles.size() times. **/
		for(int i = 0; i < particles.size(); i++)
		{
			/** Calculate the place getting landed on. **/
			float ref_weight = (r + i) / particles.size();
			
			/** Find out which particle is getting landed on. **/
			while(ref_weight > cur_weight)
			{
				cur_id++;
				cur += particles[cur_id].weight;
			}
			
			new_particles.insert(particles[i]);
		}
		
		/** Make the new particle list effective
		 *	TODO: use pointers! **/
		float new_weight = 1.0/new_particles.size();
		for(int i = 0; i < particles.size(); i++) new_particles[i].weight = new_weight;
		particles = new_particles;
	}
};

#endif
