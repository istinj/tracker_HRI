#include "obstacle.h"
using namespace std;

Obstacle::Obstacle()
{
	_seen = false;
	_o_distance = 0;

	_projected_obs_pos.setZero();
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

Eigen::Vector2f Obstacle::projectPos(const Eigen::Matrix3f& K)
{
	// any height will be ok, since we are interested only to the u-coord
	//! TODO: z-component = _obstacle->getDistance().x() ??
	Eigen::Vector3f model_point(_o_state.mean.y(), 0.0f, _o_state.mean.x());
	Eigen::Vector3f temp = K * model_point;
	Eigen::Vector2f pp(temp.x()/temp.z(), temp.y()/temp.z());
	if(pp.x() < 0)
		pp.x() = 0.0f;
	else if (pp.x() > 320.0f)
		pp.x() = 320.0f;
	_projected_obs_pos = pp;
	return pp;
}

Eigen::Vector2f Obstacle::getProjPos(void)
{
	if(_projected_obs_pos.norm() == 0)
		cerr << "No projection available yet" << endl;
	return _projected_obs_pos;

}
