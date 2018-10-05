/*
* complementary_filter.cpp
*
* Created: 05.11.2016 14:05:54
*  Author: Lukas
*/


#include "complementary_filter.h"

void complementaryFilter(float dT, float delta_angle_x_gyro, float delta_angle_y_gyro, float angle_x_accel, float angle_y_accel, float c_filter_t, float& angle_x_filtered, float& angle_y_filtered)  {
	static float c_filter_gain;
	static float angle_x_gyro_cF;
	//static float angle_y_gyro_cF;

	c_filter_gain = c_filter_t / (c_filter_t + dT);

	angle_x_gyro_cF = delta_angle_x_gyro + angle_x_filtered;
	//angle_y_gyro_cF = delta_angle_y_gyro + angle_y_filtered;

	angle_x_filtered = c_filter_gain * angle_x_gyro_cF + (1 - c_filter_gain) * angle_x_accel;
	//angle_y_filtered = c_filter_gain * angle_y_gyro_cF + (1 - c_filter_gain) * angle_y_accel;
}