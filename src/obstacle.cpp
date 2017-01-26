#include "obstacle.h"
using namespace std;

Obstacle::Obstacle()
{
	_o_distance = 0;
	_seen = false;
}

void Obstacle::setObservation(const Eigen::Vector2f& pos,
		const Eigen::Matrix2f& cov)
{
	_o_meas.mean = pos;
	_o_meas.cov = cov;
}

void Obstacle::initObs(const Eigen::Vector4f& x,
		const Eigen::Matrix4f& omega)
{
	_o_state.mean = x;
	_o_state.cov = omega;
}

void Obstacle::updateState(const State& new_state)
{
	_o_state = new_state;
}

void Obstacle::evaluateDistance(void)
{
	Eigen::Vector2f temp;
	temp << _o_state.mean.x(), _o_state.mean.y();

	_o_distance = temp.norm(); // multiply by 1000.0f??
	_o_distance *= 1000.0f;
}

void Obstacle::printState(void)
{
	Eigen::Matrix<float, 1, 4> temp = _o_state.mean.transpose();
	cout << "X     = " << temp << endl;
	cout << "Omega =\n" << _o_state.cov << endl;
}
