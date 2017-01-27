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

	//! Projects the mean point of the obstacle
	Eigen::Vector2f projectPos(const Eigen::Matrix3f& K);
	Eigen::Vector2f getProjPos(void);

	inline void setSeenFlag(void){_seen = true;};
	inline int getDistance(void){return _o_distance;};
	inline bool getFlag(void){return _seen;};
	inline Eigen::Vector2f getPos(void){return _o_state.mean.block<2,1>(0,0);};
	inline State getState(void){return _o_state;};
	inline Observation getObs(void){return _o_meas;};

private:
	State _o_state; //position, velocity and covariance
	Observation _o_meas; //position and covariance of observation

	Eigen::Vector2f _projected_obs_pos;
	float _o_distance;
	bool _seen;
};
