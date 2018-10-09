#ifndef __BALL_TRACKER_KALMAN_FILTER_H__
#define __BALL_TRACKER_KALMAN_FILTER_H__

#include <math/ExtendedKalmanFilter.h>
#include <cmath>
#include <vector>
#include <chrono>

#define STATE_DIM 2
#define SENSOR_DIM 2
#define ACTION_DIM 1
#define G_ACC 9806

class BallTrackerKalmanFilter : public ExtendedKalmanFilter<STATE_DIM, SENSOR_DIM, ACTION_DIM> {
private:
	double vel_x, vel_y;
	double prev_x, prev_y;
	double lambda_friction;
	double duration;
	double momentum;
	std::chrono::time_point<std::chrono::system_clock> prev_time;

	double signum(double value) {
		return value > 0.0 ? 1.0 : -1.0;
	}
public:

	BallTrackerKalmanFilter() {
		momentum = 0.9;
		lambda_friction = 0.2;
		vel_x = 0.0;
		vel_y = 0.0;
		prev_time = std::chrono::system_clock::now();

		this->set_R_matrix({{1000.0, 0.0}, {0.0, 1000.0}});
		this->set_Q_matrix({{1000.0, 0.0}, {0.0, 0.001}});
		this->sigma_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
	}

	void mean_prediction(Eigen::Matrix<double, ACTION_DIM, 1> u_t) {

		assert(STATE_DIM == 2);

		double acc = lambda_friction * G_ACC;
		this->mu_t_bar(0, 0) = this->mu_t(0, 0) + vel_x * duration - 0.5 * signum(vel_x) * acc * duration * duration;
		this->mu_t_bar(1, 0) = this->mu_t(1, 0) + vel_y * duration - 0.5 * signum(vel_y) * acc * duration * duration;

	}

	Eigen::Matrix<double, SENSOR_DIM, 1> mean_measure() {
		Eigen::Matrix<double, SENSOR_DIM, 1> mean_z_t;

		assert(SENSOR_DIM == 2);
		assert(STATE_DIM == 2);

		mean_z_t(0, 0) = this->mu_t_bar.norm();
		mean_z_t(1, 0) = atan2(this->mu_t_bar(1, 0), this->mu_t_bar(0, 0));

		return mean_z_t;
	}

	void jac_prediction() {
		G_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
	}

	void jac_correction() {
		assert(STATE_DIM == 2);
		assert(SENSOR_DIM == 2);

		double sq_norm = this->mu_t_bar.squaredNorm();
		double norm = this->mu_t_bar.norm();

		this->H_t(0, 0) = this->mu_t_bar(0, 0) / norm;
		this->H_t(0, 1) = this->mu_t_bar(1, 0) / norm;
		this->H_t(1, 0) = -1.0 * this->mu_t_bar(1, 0) / sq_norm;
		this->H_t(1, 1) = this->mu_t_bar(0, 0) / sq_norm;
	}


	void processFrame(double distance, double bearing, double &smooth_distance, double &smooth_bearing) {

		auto curr_time = std::chrono::system_clock::now();
	    chrono::duration<double> diff = curr_time - prev_time;
	    duration = diff.count();
	    prev_time = curr_time;

	    if(duration > 0.5) {
	    	smooth_distance = -1.0;
	    	smooth_bearing = -1.0;
	    	return;
	    }

		this->updateBelief({0.0}, {distance, bearing});

		vector<double> kalman_state = this->get_mu();
		double smoothed_x = kalman_state[0];
		double smoothed_y = kalman_state[1];

		double v_x = (smoothed_x - prev_x) / (duration + 1e-5);
		double v_y = (smoothed_y - prev_y) / (duration + 1e-5);

		vel_x = (momentum) * vel_x + v_x * (1.0 - momentum);
		vel_y = (momentum) * vel_y + v_y * (1.0 - momentum);

		prev_x = smoothed_x;
		prev_y = smoothed_y;

		smooth_distance = sqrt(smoothed_x * smoothed_x + smoothed_y * smoothed_y);
		smooth_bearing = atan2(smoothed_y, smoothed_x);

		// printf("---> LM::pF x: %8.4f   y: %8.4f   vel_x: %8.4f   vel_y: %8.4f   dur: %8.4f\n", smoothed_x, smoothed_y, vel_x, vel_y, duration);
	}
};

#endif