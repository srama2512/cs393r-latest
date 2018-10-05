#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

class KalmanFilter {
private:
  using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;
  using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  unsigned int stateDim;
  unsigned int sensorDim;
  unsigned int actionDim;

  Matrix A_t, B_t, C_t;
  Matrix R_t, Q_t;
  Vector mu_t;
  Matrix sigma_t;

  Vector mu_t_bar;
  Matrix sigma_t_bar;

	void prediction(Vector u_t);
	void correction(Vector z_t);
	void initBelief();

public:
	KalmanFilter();
	void updateBelief(std::vector<double> u_t, std::vector<double> z_t);
	// TODO: getters
	std::vector<double> get_mu();
	std::vector<double> get_sigma();
	void set_A_matrix(std::vector<std::vector<double> > A);
};

#endif