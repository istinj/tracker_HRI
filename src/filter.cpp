#include "filter.h"

ParticleFilter::ParticleFilter()
{
	_image_height = 480;
	_image_width = 640;
	_num_particles = 500;
}

void ParticleFilter::uniformSampling( void )
{
	//! Uniform sampling in the image to create state samples.
	cv::RNG rng;
	Eigen::Vector2f sum;
	sum.setZero();
	for(int i = 0; i < _num_particles; i++)
	{
		_state.samples.push_back(Eigen::Vector2f(rng.uniform(0,_image_width),
				rng.uniform(0,_image_height)));
		sum += Eigen::Vector2f(rng.uniform(0,_image_width), rng.uniform(0,_image_height));
	}
}
