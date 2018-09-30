/*
* ema_filter.cpp
*
* Created: 30.09.2018 11:45:00
*  Author: Lukas
*/


#include "ema_filter.h"

float ema_filter(float current_value, float previous_value, const float EMA_ALPHA) {
	static float filtered_value; 
	
	filtered_value = EMA_ALPHA * current_value + (1 - EMA_ALPHA) * previous_value;
	
	return filtered_value;
}