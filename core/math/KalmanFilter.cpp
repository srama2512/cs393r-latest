#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
	stateDim = 2;
	sensorDim = 1;
	actionDim = 2;

	A_t = Matrix(stateDim, stateDim);
	A_t << 1.0, 2.0, 0.0, 1.0;
	B_t = Matrix::Zero(stateDim, actionDim);
	C_t = Matrix::Identity(sensorDim, stateDim);

	R_t = 0.1 * Matrix::Identity(stateDim, stateDim);
	Q_t = 0.1 * Matrix::Identity(sensorDim, sensorDim);
	R_t(1, 1) = 0.0;
	initBelief();
	// std::cout << "A_t" << std::endl;
	// std::cout << A_t << std::endl;
	// std::cout << "B_t" << std::endl;
	// std::cout << B_t << std::endl;
	// std::cout << "C_t" << std::endl;
	// std::cout << C_t << std::endl;
	// std::cout << "R_t" << std::endl;
	// std::cout << R_t << std::endl;
	// std::cout << "Q_t" << std::endl;
	// std::cout << Q_t << std::endl;
}

void KalmanFilter::initBelief() {
	mu_t = Vector::Zero(stateDim);
	sigma_t = Matrix::Identity(stateDim, stateDim);
	sigma_t(1, 1) = 0.0;
	// std::cout << "mu_t" << std::endl;
	// std::cout << mu_t << std::endl;
	// std::cout << "sigma_t" << std::endl;
	// std::cout << sigma_t << std::endl;
}

void KalmanFilter::prediction(Vector u_t) {
	mu_t_bar = A_t * mu_t + B_t * u_t;
	sigma_t_bar = A_t * sigma_t * A_t.transpose() + R_t;
	// std::cout << "mu_t_bar" << std::endl;
	// std::cout << mu_t_bar << std::endl;
	// std::cout << "sigma_t_bar" << std::endl;
	// std::cout << sigma_t_bar << std::endl;
}

void KalmanFilter::correction(Vector z_t) {
	Matrix CSCQ_inv = (C_t * sigma_t_bar * C_t.transpose() + Q_t).inverse();
	// std::cout << "CSCQ_inv" << std::endl;
	// std::cout << CSCQ_inv << std::endl;
	Matrix K_t = sigma_t_bar * C_t.transpose() * CSCQ_inv;
	// std::cout << "K_t" << std::endl;
	// std::cout << K_t << std::endl;
	mu_t = mu_t_bar + K_t * (z_t - C_t * mu_t_bar);
	sigma_t = (Matrix::Identity(stateDim, stateDim) - K_t * C_t) * sigma_t_bar;
	std::cout << "mu_t" << std::endl;
	std::cout << mu_t << std::endl;
	std::cout << "sigma_t" << std::endl;
	std::cout << sigma_t << std::endl;
}

void KalmanFilter::updateBelief(std::vector<double> u_t_, std::vector<double> z_t_) {
	Eigen::Map<Vector> u_t(u_t_.data(), u_t_.size());
	Eigen::Map<Vector> z_t(z_t_.data(), z_t_.size());

	prediction(u_t);
	correction(z_t);
}

std::vector<double> KalmanFilter::get_mu() {
	std::vector<double> mu_t_(mu_t.data(), mu_t.data() + mu_t.size());
	return mu_t_;
}

std::vector<double> KalmanFilter::get_sigma() {
	std::vector<double> sigma_t_(sigma_t.data(), sigma_t.data() + sigma_t.rows()*sigma_t.cols());
	return sigma_t_;
}

void KalmanFilter::set_A_matrix(std::vector<std::vector<double> > A) {
	for (int i = 0; i < A.size(); ++i)
		for (int j = 0; j < A[i].size(); ++j)
			A_t(i, j) = A[i][j];
}
