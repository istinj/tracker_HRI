//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core.hpp>

#include "utilities.h"

typedef Eigen::Matrix<float, 2, 4> Matrix2_4f;
typedef Eigen::Matrix<float, 4, 2> Matrix4_2f;

//! STATO = (x,y,Vx,Vy) -> laserscan space
struct State
{
	Eigen::Vector4f mean; // (x,y,vx,vy)
	Eigen::Matrix4f cov;
};

//! OBSERVATION = (x,y) of the laserscan detection's center.
struct Observation
{
	Eigen::Vector2f mean; // (x,y)
	Eigen::Matrix2f cov;
};

class KalmanFilter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	KalmanFilter(); //ctor

	void oneStep(Eigen::Vector4f& state_mean, Eigen::Matrix4f& state_covariance);

	void setState(const Eigen::Vector4f& mean, const Eigen::Matrix4f& cov);
	void setObservation(const Eigen::Vector2f& mean, const Eigen::Matrix2f& cov);

private:
	void predict(void);
	void update(void);

	float _deltaT;

	State _state, _predicted_state;
	Observation _observation;
	Eigen::Matrix4f _transition_model;
	Matrix2_4f _obs_model;
};
