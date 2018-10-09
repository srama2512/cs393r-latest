#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

template<int N, int K, int A>
class ExtendedKalmanFilter {
private:
	void initBelief();
	void cov_prediction();
	void correction(Eigen::Matrix<double, K, 1> z_t);

public:
	Eigen::Matrix<double, N, N> G_t;
	Eigen::Matrix<double, N, A> B_t;
	Eigen::Matrix<double, K, N> H_t;

	Eigen::Matrix<double, N, N> R_t;
	Eigen::Matrix<double, K, K> Q_t;

	Eigen::Matrix<double, N, 1> mu_t;
	Eigen::Matrix<double, N, N> sigma_t;

	Eigen::Matrix<double, N, 1> mu_t_bar;
	Eigen::Matrix<double, N, N> sigma_t_bar;

	ExtendedKalmanFilter();
	void updateBelief(std::vector<double> u_t_, std::vector<double> z_t_);

	// TODO: getters
	std::vector<double> get_mu();
	std::vector<double> get_sigma();

	void set_A_matrix(std::vector<std::vector<double> > A_mat);
	void set_R_matrix(std::vector<std::vector<double> > R_mat);
	void set_Q_matrix(std::vector<std::vector<double> > Q_mat);
	void set_sigma_matrix(std::vector<std::vector<double> > sigma_mat);

	virtual void mean_prediction(Eigen::Matrix<double, A, 1> u_t) = 0;
	virtual Eigen::Matrix<double, K, 1> mean_measure() = 0;
	virtual void jac_prediction() = 0;
	virtual void jac_correction() = 0;
	
};

/*****************************
		DEFINITIONS
*****************************/

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::cov_prediction() {
	sigma_t_bar = G_t * sigma_t * G_t.transpose() + R_t;
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::correction(Eigen::Matrix<double, K, 1> z_t) {
	Eigen::Matrix<double, K, K> CSCQ_inv = (H_t * sigma_t_bar * H_t.transpose() + Q_t).inverse();
	Eigen::Matrix<double, N, K> K_t = sigma_t_bar * H_t.transpose() * CSCQ_inv;
	mu_t = mu_t_bar + K_t * (z_t - mean_measure());
	sigma_t = (Eigen::Matrix<double, N, N>::Identity() - K_t * H_t) * sigma_t_bar;
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::initBelief() {
	mu_t = Eigen::Matrix<double, N, 1>::Zero();
	sigma_t = Eigen::Matrix<double, N, N>::Identity();
}

template<int N, int K, int A>
ExtendedKalmanFilter<N, K, A>::ExtendedKalmanFilter() {

	G_t = Eigen::Matrix<double, N, N>();
	B_t = Eigen::Matrix<double, N, A>::Zero();
	H_t = Eigen::Matrix<double, K, N>::Identity();

	initBelief();
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::updateBelief(std::vector<double> u_t_, std::vector<double> z_t_) {

	Eigen::Map<Eigen::Matrix<double, A, 1> > u_t(u_t_.data(), u_t_.size());
	Eigen::Map<Eigen::Matrix<double, K, 1> > z_t(z_t_.data(), z_t_.size());

	mean_prediction(u_t);
	cov_prediction();
	correction(z_t);
}

template<int N, int K, int A>
std::vector<double> ExtendedKalmanFilter<N, K, A>::get_mu() {
	std::vector<double> mu_t_(mu_t.data(), mu_t.data() + mu_t.size());
	return mu_t_;
}

template<int N, int K, int A>
std::vector<double> ExtendedKalmanFilter<N, K, A>::get_sigma() {
	std::vector<double> sigma_t_(sigma_t.data(), sigma_t.data() + sigma_t.rows()*sigma_t.cols());
	return sigma_t_;
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::set_A_matrix(std::vector<std::vector<double> > A_mat) {
	for (int i = 0; i < A_mat.size(); ++i)
		for (int j = 0; j < A_mat[i].size(); ++j)
			G_t(i, j) = A_mat[i][j];
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::set_sigma_matrix(std::vector<std::vector<double> > sigma_mat) {
	for (int i = 0; i < sigma_mat.size(); ++i)
		for (int j = 0; j < sigma_mat[i].size(); ++j)
			sigma_t(i, j) = sigma_mat[i][j];
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::set_R_matrix(std::vector<std::vector<double> > R_mat) {
	for (int i = 0; i < R_mat.size(); ++i)
		for (int j = 0; j < R_mat[i].size(); ++j)
			R_t(i, j) = R_mat[i][j];
}

template<int N, int K, int A>
void ExtendedKalmanFilter<N, K, A>::set_Q_matrix(std::vector<std::vector<double> > Q_mat) {
	for (int i = 0; i < Q_mat.size(); ++i)
		for (int j = 0; j < Q_mat[i].size(); ++j)
			Q_t(i, j) = Q_mat[i][j];
}

#endif