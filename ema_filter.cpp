/*
* ema_filter.cpp
*
* Created: 30.09.2018 11:45:00
*  Author: Lukas
*/


#include "ema_filter.h"

float ema_filter(float current_value, float previous_value, float ema_alpha) {
	static float filtered_value;
	
	filtered_value = ema_alpha * current_value + (1 - ema_alpha) * previous_value;
	
	return filtered_value;
}
