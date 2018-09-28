/*
* kalman_filter.h
*
* Created: 05.11.2016 12:09:17
* Author: Lukas
*/


#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

class KalmanFilter {
	
	//variables
	private:
	
	// proportionality values for Q (model error covariance)
	float qp_angle, qp_rate, qp_rateBias;

	// model error covariance
	//		|q_angle	0			0			|
	//	Q = |0			q_rate		0			|
	//		|0			0			q_rateBias	|
	float q_angle, q_rate, q_rateBias;

	// measurement error covariance
	//		|r_acc	0		|
	//	R =	|0		r_gyro	|
	float r_acc, r_gyro;

	// state vector
	//		|angle		|
	//	x = |rate		|
	//		|rateBias	|
	float angle, rate, rateBias;

	// transition matrix
	//		|1	dT	-dT	|
	// Phi =|0	1	-1	|
	//		|0	0	 1	|

	// measurement matrix
	//		|1	0	0|
	// H =	|0	1	0|

	// state error covariance
	//		|p1	p2	p3|
	// P =	|p4	p5	p6|
	//		|p7	p8	p9|
	float p1, p2, p3, p4, p5, p6, p7, p8, p9;
	
	// Kalman gain
	//		|k1	k2|
	// K =	|k3	k4|
	//		|k5	k6|
	float k1, k2, k3, k4, k5, k6;


	//functions
	private:

	// calculates the model error covariance with the current dT
	void calc_Q(float dT);
	
	public:
	
	// constructor
	KalmanFilter(float qp_angle_new, float qp_rate_new, float qp_rateBias_new, float r_acc_new, float r_gyro_new, float angle_new);
	
	// set starting angle
	void set_angle(float angle_new);

	float get_angle(float dT, float rate_new, float angle_new);
	float get_rate();
	float get_rateBias();

	// used to tune the Kalman filter
	void set_qp_angle(float qp_angle_new);
	void set_qp_rate(float qp_rate_new);
	void set_qp_rateBias(float qp_rateBias_new);

	void set_r_acc(float r_acc_new);
	void set_r_gyro(float r_gyro_new);

	float get_q_angle();
	float get_q_rate();
	float get_q_rateBias();

	float get_r_acc();
	float get_r_gyro();
};

#endif //__KALMAN_FILTER_H__
