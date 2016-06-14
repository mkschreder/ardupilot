#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

class RangeAvoid {
public: 
	RangeAvoid(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow); 
	
	void set_kP(float kp); 
	void set_kI(float ki); 
	void set_kD(float kd); 

	void input_desired_velocity_ms(float forward, float right); 

	void update(const Vector2f flow_rate, float dt); 

	float get_desired_pitch_angle(); 
	float get_desired_roll_angle(); 

	const Vector2f get_filtered_flow(); 

	void reset(); 
private: 
	AP_RangeFinder_6DOF *_rangefinder; 
	AP_AHRS *_ahrs; 
	AP_InertialSensor *_ins; 
	OpticalFlow *_optflow; 
	AC_PID _pitch_pid, _roll_pid; 
	LowPassFilterFloat _flow_front_filt, _flow_right_filt; 
	float _pitchComp, _rollComp; 
	float _flow_front_filtered, _flow_right_filtered; 
	float _desired_forward, _desired_right; 
}; 
