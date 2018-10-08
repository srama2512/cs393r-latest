#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

template<int N, int K, int A>
class KalmanFilter {
private:

	Eigen::Matrix<double, N, N> A_t;
	Eigen::Matrix<double, N, A> B_t;
	Eigen::Matrix<double, K, N> C_t;

	Eigen::Matrix<double, N, N> R_t;
	Eigen::Matrix<double, K, K> Q_t;

	Eigen::Matrix<double, N, 1> mu_t;
	Eigen::Matrix<double, N, N> sigma_t;

	Eigen::Matrix<double, N, 1> mu_t_bar;
	Eigen::Matrix<double, N, N> sigma_t_bar;

	void prediction(Eigen::Matrix<double, A, 1> u_t) {
		mu_t_bar = A_t * mu_t + B_t * u_t;
		sigma_t_bar = A_t * sigma_t * A_t.transpose() + R_t;
		// std::cout << "mu_t_bar" << std::endl;
		// std::cout << mu_t_bar << std::endl;
		// std::cout << "sigma_t_bar" << std::endl;
		// std::cout << sigma_t_bar << std::endl;
	}

	void correction(Eigen::Matrix<double, K, 1> z_t) {
		Eigen::Matrix<double, K, K> CSCQ_inv = (C_t * sigma_t_bar * C_t.transpose() + Q_t).inverse();
		// std::cout << "CSCQ_inv" << std::endl;
		// std::cout << CSCQ_inv << std::endl;
		Eigen::Matrix<double, N, K> K_t = sigma_t_bar * C_t.transpose() * CSCQ_inv;
		// std::cout << "K_t" << std::endl;
		// std::cout << K_t << std::endl;
		mu_t = mu_t_bar + K_t * (z_t - C_t * mu_t_bar);
		sigma_t = (Eigen::Matrix<double, N, N>::Identity() - K_t * C_t) * sigma_t_bar;
		// std::cout << "mu_t" << std::endl;
		// std::cout << mu_t << std::endl;
		// std::cout << "sigma_t" << std::endl;
		// std::cout << sigma_t << std::endl;
	}

	void initBelief() {
		mu_t = Eigen::Matrix<double, N, 1>::Zero();
		sigma_t = Eigen::Matrix<double, N, N>::Identity();
		// sigma_t(1, 1) = 0.0;
		// std::cout << "mu_t" << std::endl;
		// std::cout << mu_t << std::endl;
		// std::cout << "sigma_t" << std::endl;
		// std::cout << sigma_t << std::endl;
	}

public:

	KalmanFilter() {

		A_t = Eigen::Matrix<double, N, N>();
		// A_t << 1.0, 2.0, 0.0, 1.0;
		B_t = Eigen::Matrix<double, N, A>::Zero();
		C_t = Eigen::Matrix<double, K, N>::Identity();

		// R_t = 0.1 * Eigen::Matrix<double, N, N>::Identity();
		// Q_t = 0.1 * Eigen::Matrix<double, K, K>::Identity();
		// R_t(1, 1) = 0.0;
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

	void updateBelief(std::vector<double> u_t_, std::vector<double> z_t_) {

		Eigen::Map<Eigen::Matrix<double, A, 1> > u_t(u_t_.data(), u_t_.size());
		Eigen::Map<Eigen::Matrix<double, K, 1> > z_t(z_t_.data(), z_t_.size());

		prediction(u_t);
		correction(z_t);
	}

	// TODO: getters
	std::vector<double> get_mu() {
		std::vector<double> mu_t_(mu_t.data(), mu_t.data() + mu_t.size());
		return mu_t_;
	}

	std::vector<double> get_sigma() {
		std::vector<double> sigma_t_(sigma_t.data(), sigma_t.data() + sigma_t.rows()*sigma_t.cols());
		return sigma_t_;
	}

	void set_A_matrix(std::vector<std::vector<double> > A_mat) {
		for (int i = 0; i < A_mat.size(); ++i)
			for (int j = 0; j < A_mat[i].size(); ++j)
				A_t(i, j) = A_mat[i][j];
	}

	void set_sigma_matrix(std::vector<std::vector<double> > sigma_mat) {
		for (int i = 0; i < sigma_mat.size(); ++i)
			for (int j = 0; j < sigma_mat[i].size(); ++j)
				sigma_t(i, j) = sigma_mat[i][j];
	}

	void set_R_matrix(std::vector<std::vector<double> > R_mat) {
		for (int i = 0; i < R_mat.size(); ++i)
			for (int j = 0; j < R_mat[i].size(); ++j)
				R_t(i, j) = R_mat[i][j];
	}

	void set_Q_matrix(std::vector<std::vector<double> > Q_mat) {
		for (int i = 0; i < Q_mat.size(); ++i)
			for (int j = 0; j < Q_mat[i].size(); ++j)
				Q_t(i, j) = Q_mat[i][j];
	}
};

#endif