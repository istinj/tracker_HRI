//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>

#include "utilities.h"

struct State
{
	std::vector<Eigen::Vector2f> samples;
	Eigen::Vector2f mean;
	Eigen::Matrix4f variance;
};

struct Observation
{
	Eigen::Vector2f mean;
	Eigen::Matrix2f variance;
};

class ParticleFilter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	ParticleFilter(); //ctor

private:
	//! STATO = (u,v,Vx,Vy) -> image space
	//! OBSERVATION = (u,v) of the detection's center.
	void uniformSampling( void );
	void predict(Eigen::Vector4f samples, Eigen::Matrix4f T);

	State _state;
	Observation _observation;

	Eigen::Matrix4f _transition_model;
	std::vector<float> _weights;
	int _num_particles;
	int _image_width;
	int _image_height;
};
