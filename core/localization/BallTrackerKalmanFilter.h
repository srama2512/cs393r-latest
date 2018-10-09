#ifndef __BALL_TRACKER_KALMAN_FILTER_H__
#define __BALL_TRACKER_KALMAN_FILTER_H__

#include <math/ExtendedKalmanFilter.h>

template<int N, int K, int A>
class BallTrackerKalmanFilter : public ExtendedKalmanFilter<N, K, A> {
public:

	void mean_prediction(Eigen::Matrix<double, A, 1> u_t) {
		this->mu_t_bar = this->G_t * this->mu_t + this->B_t * u_t;
	}

	Eigen::Matrix<double, K, 1> mean_measure() {
		return this->H_t * this->mu_t_bar;
	}

	void jac_prediction() {
		return;
	}

	void jac_correction() {
		return;
	}
};

#endif