#include "detection.h"
using namespace std;

Detection::Detection()
{
	_init = false;

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

void Detection::setObservation(const Eigen::Vector2f& pos,
		const Eigen::Matrix2f& omega)
{
	_d_meas.mean = pos;
	_d_meas.cov = omega;
}

void Detection::initState(const Eigen::Vector4f& x,
		const Eigen::Matrix4f& omega)
{
	_d_state.mean = x;
	_d_state.cov = omega;
}

void Detection::updateState(void)
{
	predict();
	update();
}

void Detection::printState(void)
{
	Eigen::Matrix<float, 1, 4> temp = _d_state.mean.transpose();
	cout << "X     = " << temp << endl;
	cout << "Omega =\n" << _d_state.cov << endl;
}

void Detection::predict(void)
{
	_prediction.mean = _transition_model * _d_state.mean;
	_prediction.cov = _transition_model * _d_state.cov * _transition_model.transpose();
}

void Detection::update(void)
{
	Eigen::Vector2f h_x = _obs_model * _prediction.mean;

	Eigen::Matrix2f temp = _obs_model*_prediction.cov*_obs_model.transpose() + _d_meas.cov;
	Matrix4_2f K = _prediction.cov * _obs_model.transpose() * temp.inverse();

	_d_state.mean = _prediction.mean + K * (_d_meas.mean - h_x);
	_d_state.cov = _prediction.cov - K * _obs_model * _prediction.cov;
}
