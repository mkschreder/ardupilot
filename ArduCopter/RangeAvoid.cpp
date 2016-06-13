#include "RangeAvoid.h"

#define RANGE_MAX_RESPONSE 45.0
#define RANGE_MIN_CLEARANCE 60 
#define RANGE_AVOID_VELOCITY 0.1

RangeAvoid::RangeAvoid(AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0) {
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	_flow_front_filt.set_cutoff_frequency(5.0); 
	_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_kP(double kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp); 
}

void RangeAvoid::set_kD(double kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd); 
}

void RangeAvoid::set_kI(double ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki); 
}

void RangeAvoid::input_desired_velocity_ms(double forward, double right){
	_desired_forward = forward; 
	_desired_right = right; 
}

const Vector2f RangeAvoid::get_filtered_flow() {
	return Vector2f(_flow_front_filtered, _flow_right_filtered); 
}

void RangeAvoid::update(double dt){
	//double acc_front = _ins->get_accel().x; 
	//_pitch_pid.set_input_filter_all(acc_front); 
	//_roll_pid.set_input_filter_all(_rangefinder->get_velocity_x()); 
	
	double clear_front = _rangefinder->get_front_clearance_cm(); 
	double clear_back = _rangefinder->get_back_clearance_cm(); 
	double clear_right = _rangefinder->get_right_clearance_cm(); 
	double clear_left = _rangefinder->get_left_clearance_cm(); 

	double desired_forward = _desired_forward; 
	double desired_right = _desired_right; 

	// try to avoid object by going in opposite direction
	//if(clear_front < RANGE_MIN_CLEARANCE) desired_forward += RANGE_AVOID_VELOCITY; 
	//if(clear_back < RANGE_MIN_CLEARANCE) desired_forward -= RANGE_AVOID_VELOCITY; 
	//if(clear_left < RANGE_MIN_CLEARANCE) desired_right += RANGE_AVOID_VELOCITY; 
	//if(clear_right < RANGE_MIN_CLEARANCE) desired_right -= RANGE_AVOID_VELOCITY; 

	double flow_forward = -_optflow->flowRate().x; 
	double flow_right = _optflow->flowRate().y; 
	
	_flow_front_filt.apply(flow_forward, dt); 
	_flow_right_filt.apply(flow_right, dt); 

	_flow_front_filtered = _flow_front_filt.get();  
	_flow_right_filtered = _flow_right_filt.get();  

	_pitch_pid.set_input_filter_all(desired_forward - _flow_front_filtered); 
	_roll_pid.set_input_filter_all(desired_right - _flow_right_filtered); 
}

double RangeAvoid::get_desired_pitch_angle(){
	double front = _rangefinder->get_front_clearance_cm(); 
	double y_vel = _rangefinder->get_velocity_y(); 
	double acc_front = _ins->get_accel().x; 
	
	return constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE); 
	//if(y_vel > 0 && front < 80.0 && acc_front > 0){
//		return constrain_float(_pitch_pid.get_pid(), 0, RANGE_MAX_RESPONSE); 
	//}
	//return 0; 
}

double RangeAvoid::get_desired_roll_angle(){
	return constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE); 
}
