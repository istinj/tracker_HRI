#include "filter.h"

KalmanFilter::KalmanFilter()
{
	_deltaT = 0.05f;
	_transition_model <<
			1, 0, _deltaT, 0,
			0, 1, 0, _deltaT,
			0, 0, 1, 0,
			0, 0, 0, 1;
	_obs_model <<
			1, 0, 0, 0,
			0, 1, 0, 0;
}

void KalmanFilter::predict(void)
{
	_predicted_state.mean = _transition_model * _state.mean;
	_predicted_state.cov = _transition_model * _state.cov * _transition_model.transpose();
}

void KalmanFilter::update(void)
{
	Eigen::Vector2f h_x = _obs_model * _predicted_state.mean;

	Eigen::Matrix2f temp = _obs_model*_predicted_state.cov*_obs_model.transpose() + _observation.cov;
	Matrix4_2f K = _predicted_state.cov * _obs_model.transpose() * temp.inverse();

	_state.mean = _predicted_state.mean + K * (_observation.mean - h_x);
	_state.cov = _predicted_state.cov - K * _obs_model * _predicted_state.cov;
}

void KalmanFilter::oneStep(Obstacle* obstacle)
{
	_state = obstacle->getState();
	_observation = obstacle->getObs();

	predict();
	update();

	_state_history_vec.push_back(_state);
	obstacle->updateState(_state);
}
