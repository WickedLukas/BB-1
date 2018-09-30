/*
* ema_filter.h
*
* Created: 30.09.2018 11:45:00
*  Author: Lukas
*/


#ifndef __EMA_FILTER_H__
#define __EMA_FILTER_H__

float ema_filter(float current_value, float previous_value, const float EMA_ALPHA);

#endif //__EMA_FILTER_H__
