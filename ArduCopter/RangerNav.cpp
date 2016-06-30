#include "RangerNav.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

RangerNav::RangerNav(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro){
	_ahrs = ahrs; 
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_baro = baro; 

	kf_range_x.set_sensor_noise_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.05, 0.0, 0.0, 0.0},
		{0.0, 0.1, 0.0, 0.0},
		{0.0, 0.0, 0.4, 0.0},
		{0.0, 0.0, 0.0, 0.4}
	})); 

	kf_range_x.set_process_noise_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.0001, 0.0, 0.0, 0.0},
		{0.0, 0.0001, 0.0, 0.0},
		{0.0, 0.0, 0.0001, 0.0},
		{0.0, 0.0, 0.0, 0.0001}
	})); 

	kf_range_x.set_state_input_matrix(math::Matrix<4, 4>((const float[4][4]){
		{1.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}
	})); 
}

void RangerNav::update(float dt){
	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;

	// read rangefinder
	float front, back, right, left, bottom, top; 
	float dfront, dback, dright, dleft, dbottom, dtop; 
	static float pdfront = 0, pdback = 0, pdright = 0, pdleft = 0, pdbottom = 0, pdtop = 0; 
	_rangefinder->update(dt); 
	_rangefinder->get_readings_m(&front, &back, &right, &left, &bottom, &top); 
	_rangefinder->get_rates_mps(&dfront, &dback, &dright, &dleft, &dbottom, &dtop); 

	const math::Vector<4> &p = kf_range_x.get_prediction(); 

	// update rangefinder filters
	const float vfb[4] = {
		(abs(p(1)) < 0.05)?(p(0) * 0.95):p(0), back, dfront, dback
	}; 
	math::Vector<4> fb(vfb); 

	float fmat[4][4] = {
		{1.0, 0.0, 0.5*dt, -0.5*dt},
		{0.0, 0.0, 0.5,    -0.5},
		{0.0, 0.0, 1.0,    0.0},
		{0.0, 0.0, 0.0,    1.0}
	}; 
	kf_range_x.set_state_transition_matrix(math::Matrix<4,4>(fmat)); 
	kf_range_x.update(fb, math::Vector<4>(), dt); 

	hal.console->printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
		front, back, dfront, dback, p(0), p(1), p(2), p(3)); 

	// update navigation filters
}

Vector3f RangerNav::get_velocity() {
	const math::Vector<4> &p = kf_range_x.get_prediction(); 
	return Vector3f(p(0), 0, 0); 
}
