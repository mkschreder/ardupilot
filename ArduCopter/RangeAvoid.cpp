#include "RangeAvoid.h"

#define RANGE_MAX_RESPONSE 45.0
#define RANGE_MIN_CLEARANCE 60.0 
#define RANGE_AVOID_RESPONSE_SCALE 1.0

RangeAvoid::RangeAvoid(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0) {
	_ahrs = ahrs; 
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	_flow_front_filt.set_cutoff_frequency(5.0); 
	_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_kP(float kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp); 
}

void RangeAvoid::set_kD(float kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd); 
}

void RangeAvoid::set_kI(float ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki); 
}

void RangeAvoid::input_desired_velocity_ms(float forward, float right){
	_desired_forward = forward; 
	_desired_right = right; 
}

const Vector2f RangeAvoid::get_filtered_flow() {
	return Vector2f(_flow_front_filtered, _flow_right_filtered); 
}

void RangeAvoid::reset(){
	_pitch_pid.reset_I(); 
	_roll_pid.reset_I(); 
}

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void RangeAvoid::update(const Vector2f flow_rate, float dt){
	float clear_front = _rangefinder->get_front_clearance_cm(); 
	float clear_back = _rangefinder->get_back_clearance_cm(); 
	float clear_right = _rangefinder->get_right_clearance_cm(); 
	float clear_left = _rangefinder->get_left_clearance_cm(); 
	float clear_bottom = _rangefinder->get_bottom_clearance_cm(); 

	float desired_forward = _desired_forward; 
	float desired_right = _desired_right; 

	if(!is_zero(_desired_forward)) _flow_distance_front = 0; 
	if(!is_zero(_desired_right)) _flow_distance_right = 0; 

	// try to avoid object by going in opposite direction
	Vector2f directions[4] = {Vector2f(clear_front, 0), Vector2f(-clear_back, 0), Vector2f(0, -clear_left), Vector2f(0, clear_right)};
	Vector2f closest_point(1000, 1000); 
	bool found = false; 
	for(int c = 0; c < 4; c++){
		double len = directions[c].length(); 
		if(len < closest_point.length() && len < RANGE_MIN_CLEARANCE) {
			closest_point = directions[c]; 
			found = true; 
		}
	}

	// get the response vector which is same as closest point but with inverted length and pointing in the other direction
	if(found){
		float len = closest_point.length() / RANGE_MIN_CLEARANCE; 
		closest_point.normalize(); 
		closest_point *= (1.0 - len) * RANGE_AVOID_RESPONSE_SCALE;  
		//desired_forward -= constrain_float(closest_point.x, -0.5, 0.5); 
		//desired_right -= constrain_float(closest_point.y, -0.5, 0.5); 
	}

	float flow_forward = flow_rate.x * clear_bottom * 0.01; 
	float flow_right = flow_rate.y * clear_bottom * 0.01; 
	
	_flow_front_filt.apply(flow_forward, dt); 
	_flow_right_filt.apply(flow_right, dt); 

	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = accel.x - sin(_ahrs->pitch) * accel_len; 
	accel.y = accel.y + sin(_ahrs->roll) * accel_len;

	float w = (0.075 / (0.075 + dt)); 
	//_flow_front_filtered += ((flow_forward - _flow_front_filtered) * w - accel.x * (1.0 - w)) * dt; //_flow_front_filt.get();  
	//_flow_right_filtered += ((flow_right - _flow_right_filtered) * w + accel.y * (1.0 - w)) * dt; //flow_right; //_flow_right_filt.get();  
	_flow_front_filtered = w * (_flow_front_filtered + accel.x * dt) + (1.0 - w) * flow_forward;  
	_flow_right_filtered = w * (_flow_right_filtered + accel.y * dt) + (1.0 - w) * flow_right;  
	
	_flow_distance_front += _flow_front_filtered * dt; 
	_flow_distance_right += _flow_right_filtered * dt; 

	hal.console->printf("%f %f %f %f %f %f %f %f %f %f %f\n", clear_front, clear_back, _flow_distance_front, _flow_distance_right, clear_bottom, desired_forward, desired_right, 
			_flow_front_filtered, _flow_right_filtered, _rangefinder->get_velocity_forward() / 100.0, _rangefinder->get_velocity_right() / 100.0); 
	//, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 
	//hal.console->printf("%f %f %f %f %f %f %f %f %f\n", flow_rate.x, flow_rate.y, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 

	_pitch_pid.set_input_filter_all(desired_forward - _flow_front_filtered); 
	_roll_pid.set_input_filter_all(desired_right - _flow_right_filtered); 
}

float RangeAvoid::get_desired_pitch_angle(){
	return -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE); 
}

float RangeAvoid::get_desired_roll_angle(){
	return constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE); 
}
