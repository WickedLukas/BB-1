/*
* PID_controller.cpp
*
* Created: 06.01.2017 11:01:56
* Author: Lukas
*/


#include "PID_controller.h"

// constructor
PID_controller::PID_controller(float K_p_new, float K_i_new, float K_d_new, float min_mv_tol_new, float min_mv_new, float max_mv_new) {
	mv = 0;
	
	K_p = K_p_new;
	K_i = K_i_new;
	K_d = K_d_new;
	
	min_mv_tol = abs(min_mv_tol_new);
	min_mv = abs(min_mv_new);
	max_mv = abs(max_mv_new);
	
	update_time_ratio = 1;
	pid_update_counter = 0;
	
	dT_pid = 0;
	
	proportional = 0;
	proportional_old = 0;
	integral = 0;
	integral_old = 0;
	derivative = 0;
}

// set targeted PID update time
void PID_controller::set_pid_update_time(int32_t pid_update_time_new, int32_t mpu_update_time_new) {
	pid_update_time = pid_update_time_new;
	mpu_update_time = mpu_update_time_new;
	
	update_time_ratio = round((float) pid_update_time / mpu_update_time);
	
	// update time ratio has to be one at least
	if (update_time_ratio <= 1) {
		update_time_ratio = 1;
	}
}

// set PID gains
void PID_controller::set_K_p(float K_p_new) {
	K_p = K_p_new;
}
void PID_controller::set_K_i(float K_i_new) {
	K_i = K_i_new;
}
void PID_controller::set_K_d(float K_d_new) {
	K_d = K_d_new;
}

// set tolerance before minimum min_mv is applied
void PID_controller::set_min_mv_tol(float min_mv_tol_new) {
	min_mv_tol = abs(min_mv_tol_new);
}
// set absolute minimum for manipulated variable (mv)
void PID_controller::set_min_mv(float min_mv_new) {
	min_mv = abs(min_mv_new);
}
// set absolute maximum for manipulated variable (mv)
void PID_controller::set_max_mv(float max_mv_new) {
	max_mv = abs(max_mv_new);
}

// calculate manipulated variable (mv) from setpoint (sp) and process variable (pv)
float PID_controller::get_mv(float sp, float pv, float dT) {
	pid_update_counter++;
	
	dT_pid += dT;
	
	if (pid_update_counter == update_time_ratio) {
		proportional = (sp - pv);
		
		prop_sum = proportional + proportional_old;
		
		integral += dT_pid * prop_sum / 2;
		
		// wind-up compensation
		if (((mv >= max_mv) && (prop_sum > 0)) || ((mv <= -max_mv) && (prop_sum < 0)))
		{
			integral = integral_old;
		}
		
		derivative = (proportional - proportional_old) / dT_pid;
		
		mv = K_p * proportional + K_i * integral + K_d * derivative;
		
		// constrain the manipulated variable (mv)
		constrain_mv();
		
		pid_update_counter = 0;
		dT_pid = 0;
		proportional_old = proportional;
		integral_old = integral;
	}
	return mv;
}

// constrain manipulated variable (mv)
void PID_controller::constrain_mv() {
	// Increase the manipulated variable |mv| to the minimum (example: the minimum at which the motors start rotating).
	// If the control variable is within (-min_mv_tol, min_mv_tol), it is set to zero (example: to avoid permanent switching between negative and positive minimum values).
	if (mv <= -min_mv_tol) {
		mv -= min_mv;
	}
	else if (mv >= min_mv_tol) {
		mv += min_mv;
	}
	else {
		mv = 0;
		return;
	}
	
	// limit manipulated variable (example: maximum motor speed)
	mv = constrain(mv, -max_mv, max_mv);
}

// reset PID controller
void PID_controller::reset() {
	mv = 0;
	
	pid_update_counter = 0;
	
	dT_pid = 0;
	
	proportional = 0;
	proportional_old = 0;
	integral = 0;
	integral_old = 0;
	derivative = 0;
}
