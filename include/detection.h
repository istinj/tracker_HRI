#pragma once
//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "utilities.h"

class Detection
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	Detection();
	void initState(const Eigen::Vector4f& x, const Eigen::Matrix4f& omega);
	void setObservation(const Eigen::Vector2f& pos, const Eigen::Matrix2f& omega);
	void updateState(void);
	void printState(void);

	inline void setInitFlag(void){_init = true;};
	inline bool getInitFlag(void){return _init;};
	inline Eigen::Vector2f getPos(void){return _d_state.mean.block<2,1>(0,0);};
	inline State getState(void){return _d_state;};
	inline Observation getObs(void){return _d_meas;};

private:
	void predict(void);
	void update(void);

	State _d_state, _prediction;//position, velocity and covariance
	Observation _d_meas;		//position and covariance of observation

	Eigen::Matrix4f _transition_model;
	Matrix2_4f _obs_model;

	std::vector<State> _state_history_vec;


	bool _init;
	float _deltaT;
};
