#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

class RangeAvoid {
public: 
	RangeAvoid(AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow); 
	
	void set_kP(double kp); 
	void set_kI(double ki); 
	void set_kD(double kd); 

	void input_desired_velocity_ms(double forward, double right); 

	void update(double dt); 

	double get_desired_pitch_angle(); 
	double get_desired_roll_angle(); 

	const Vector2f get_filtered_flow(); 
private: 
	AP_RangeFinder_6DOF *_rangefinder; 
	AP_InertialSensor *_ins; 
	OpticalFlow *_optflow; 
	AC_PID _pitch_pid, _roll_pid; 
	LowPassFilterFloat _flow_front_filt, _flow_right_filt; 
	double _pitchComp, _rollComp; 
	double _flow_front_filtered, _flow_right_filtered; 
	double _desired_forward, _desired_right; 
}; 
