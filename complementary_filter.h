/*
* complementary_filter.h
*
* Created: 05.11.2016 14:03:24
*  Author: Lukas
*/


#ifndef __COMPLEMENTARY_FILTER_H__
#define __COMPLEMENTARY_FILTER_H__

void complementaryFilter(float& angle_x_filtered, float& angle_y_filtered, float dT, float delta_angle_x_gyro, float delta_angle_y_gyro,
float angle_x_accel, float angle_y_accel, const float C_FILTER_T);

#endif //__COMPLEMENTARY_FILTER_H__
