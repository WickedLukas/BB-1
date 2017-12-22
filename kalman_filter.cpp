/*
* kalman_filter.cpp
*
* Created: 05.11.2016 12:14:03
* Author: Lukas
*/


#include "kalman_filter.h"

// constructor
KalmanFilter::KalmanFilter(float qp_angle_new, float qp_rate_new, float qp_rateBias_new, float r_acc_new, float r_gyro_new, float angle_new) {
	qp_angle = qp_angle_new;
	qp_rate = qp_rate_new;
	qp_rateBias = qp_rateBias_new;
	
	r_acc = r_acc_new;
	r_gyro = r_gyro_new;

	angle = angle_new;
	rate = 0;
	rateBias = 0;

	// assumption: initial state is unknown
	p1 = 10;
	p2 = 0;
	p3 = 0;
	p4 = 0;
	p5 = 10;
	p6 = 0;
	p7 = 0;
	p8 = 0;
	p9 = 10;
}

// updates the model error covariance matrix with the current dT
void KalmanFilter::calc_Q(float dT) {
	q_angle		= qp_angle * dT * dT;
	q_rate		= qp_rate * dT;
	q_rateBias	= qp_rateBias * dT;
}

// calculates the Kalman filtered angle (1-D)
float KalmanFilter::get_angle(float dT, float rate_new, float angle_new) {
	static float angle_filtered;

	static float det;
	static float z_angle, z_rate;

	calc_Q(dT);

	//--------------------------------Update--------------------------------

	// Kalman gain
	// K = P(k) * H' * (H * P(k) * H' + R)^(-1)
	det = (p1 + r_acc) * (p5 + r_gyro) - p2 * p4;

	k1 = (p1 * (p5 + r_gyro) - p2 * p4) / det;
	k2 = (p2 * (p1 + r_acc) - p1 * p2) / det;
	k3 = (p4 * (p5 + r_gyro) - p4 * p5) / det;
	k4 = (p5 * (p1 + r_acc) - p2 * p4) / det;
	k5 = (p7 * (p5 + r_gyro) - p4 * p8) / det;
	k6 = (p8 * (p1 + r_acc) - p2 * p7) / det;

	// update state
	// x(k) = x(k) + K(k) * (y - H * x(k)) with z = (y - H * x(k))
	z_angle = angle_new - angle;
	z_rate = rate_new - rate;

	angle += k1 * z_angle + k2 * z_rate;
	rate += k3 * z_angle + k4 * z_rate;
	rateBias += k5 * z_angle + k6 * z_rate;
	
	// filtered angle
	angle_filtered = angle;
	
	// update state error
	// P(k) = (I - K(k) * H) * P(k)
	p1 = (1 - k1) * p1 - k2 * p4;
	p2 = (1 - k1) * p2 - k2 * p5;
	p3 = (1 - k1) * p3 - k2 * p6;
	p4 = (1 - k4) * p4 - k3 * p1;
	p5 = (1 - k4) * p5 - k3 * p2;
	p6 = (1 - k4) * p6 - k3 * p3;
	p7 = -k5 * p1 - k6 * p4 + p7;
	p8 = -k5 * p2 - k6 * p5 + p8;
	p9 = -k5 * p3 - k6 * p6 + p9;

	//------------------------------Prediction------------------------------

	// predict state
	// x(k+1) = Phi * x(k)
	angle += dT * (rate_new - rateBias);

	// predict state error
	// P(k+1) = Phi * P(k) * Phi' + Q
	p1 += dT * (p4 - p7 + p2 - p3) + dT * dT * (p5 - p8 - p6 + p9) + q_angle;
	p2 += dT * (p5 - p8);
	p3 += dT * (p6 - p9);
	p4 += dT * (p5 - p6);
	p5 += q_rate;
	p7 += dT * (p8 - p9);
	p9 += q_rateBias;

	return angle_filtered;
}

// set starting angle
void KalmanFilter::set_angle(float angle_new) {
	angle = angle_new;
};

float KalmanFilter::get_rate() {
	return rate;
};

float KalmanFilter::get_rateBias() {
	return rateBias;
};


// used to tune the Kalman filter
void KalmanFilter::set_qp_angle(float qp_angle_new) {
	qp_angle = qp_angle_new;
};
void KalmanFilter::set_qp_rate(float qp_rate_new) {
	qp_rate = qp_rate_new;
};
void KalmanFilter::set_qp_rateBias(float qp_rateBias_new) {
	qp_rateBias = qp_rateBias_new;
};

void KalmanFilter::set_r_acc(float r_acc_new) {
	r_acc = r_acc_new;
};
void KalmanFilter::set_r_gyro(float r_gyro_new) {
	r_gyro = r_gyro_new;
};

float KalmanFilter::get_q_angle() {
	return q_angle;
};
float KalmanFilter::get_q_rate() {
	return q_rate;
};
float KalmanFilter::get_q_rateBias() {
	return q_rateBias;
};

float KalmanFilter::get_r_acc() {
	return r_acc;
};
float KalmanFilter::get_r_gyro() {
	return r_gyro;
};