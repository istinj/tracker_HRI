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

class Obstacle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	Obstacle(); //ctor

	void setObservation(const Eigen::Vector2f& pos, const Eigen::Matrix2f& cov);
	void initObs(const Eigen::Vector4f& x, const Eigen::Matrix4f& omega);
	void updateState(const State& new_state);
	void evaluateDistance(void);

	void printState(void);

	inline void getState(State& state){state = _o_state;};
	inline void getObs(Observation& obs){obs = _o_meas;};
	inline void setSeenFlag(void){_seen = true;};
	inline int getDistance(void){return _o_distance;};
	inline bool getFlag(void){return _seen;};
	inline Eigen::Vector2f getPos(void){return _o_state.mean.block<2,1>(0,0);};

private:
	State _o_state; //position, velocity and covariance
	Observation _o_meas; //position and covariance of observation

	float _o_distance;
	bool _seen;
};
