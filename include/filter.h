#pragma once
//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>

#include "utilities.h"
#include "obstacle.h"

typedef Eigen::Matrix<float, 2, 4> Matrix2_4f;
typedef Eigen::Matrix<float, 4, 2> Matrix4_2f;

class KalmanFilter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	KalmanFilter(); //ctor

	void oneStep(Obstacle* obstacle);
	void setState(const Eigen::Vector4f& mean, const Eigen::Matrix4f& cov);
	void setObservation(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov);

	inline int getHistorySize(void){return _state_history_vec.size();};

private:
	void predict(void);
	void update(void);

	float _deltaT;

	State _state, _predicted_state;
	Observation _observation;
	Eigen::Matrix4f _transition_model;
	Matrix2_4f _obs_model;

	std::vector<State> _state_history_vec;
};
