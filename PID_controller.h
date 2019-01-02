/*
* PID_controller.h
*
* Created: 06.01.2017 11:01:56
* Author: Lukas
*/


#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "Arduino.h"

class PID_controller
{
	//variables
	private:

	// manipulated variable (mv)
	float mv;
	
	// proportional gain
	float K_p;
	// integral gain
	float K_i;
	// derivative gain
	float K_d;
	
	// tolerance before minimum min_mv is applied
	float min_mv_tol;
	// absolute minimum of manipulated variable (mv)
	float min_mv;
	// absolute maximum of manipulated variable (mv)
	float max_mv;
	
	// targeted PID update time in microseconds
	int32_t pid_update_time;
	
	// targeted MPU update time in microseconds
	int32_t mpu_update_time;
	
	// PID to MPU update time ratio
	int16_t update_time_ratio;
	// PID update counter
	int16_t pid_update_counter;
	
	// current PID time in seconds
	float dT_pid;
	
	// proportional error
	float proportional, proportional_old;
	// integral error
	float integral, integral_old;
	// derivative error
	float derivative;
	
	// prop_sum = proportional + proportional_old
	float prop_sum;
	
	
	//functions
	public:
	
	// constructor
	PID_controller(float K_p_new, float K_i_new, float K_d_new, float min_mv_tol_new, float min_mv_new, float max_mv_new);
	
	// set targeted PID update time
	void set_pid_update_time(int32_t pid_update_time_new, int32_t mpu_update_time_new);

	// set PID gains
	void set_K_p(float K_p_new);
	void set_K_i(float K_i_new);
	void set_K_d(float K_d_new);
	
	// set tolerance before minimum min_mv is applied
	void set_min_mv_tol(float min_mv_tol_new);
	// set absolute minimum for manipulated variable (mv)
	void set_min_mv(float min_mv_new);
	// set absolute maximum for manipulated variable (mv)
	void set_max_mv(float max_mv_new);

	// calculate manipulated variable (mv) from setpoint (sp) and process variable (pv)
	float get_mv(float sp_new, float pv_new, float dT);
	// constrain manipulated variable (mv)
	void constrain_mv();
	// reset PID controller
	void reset();
};

#endif //__PID_CONTROLLER_H__
