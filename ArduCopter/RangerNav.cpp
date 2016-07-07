/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "RangerNav.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

RangerNav::PVPredictor::PVPredictor(){
	_kf.set_state_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.01, 0.0, 0.0, 0.0},
		{0.0, 0.01, 0.0, 0.0},
		{0.0, 0.0, 0.01, 0.0},
		{0.0, 0.0, 0.0, 0.01}
	})); 
	_lp_velocity.set_cutoff_frequency(1.0); 
}

void RangerNav::PVPredictor::input(float flow_vel, float flow_quality, float range_pos, float range_neg, float dt){
	//_smooth_flow.update(flow_vel); 
	_median_pos.update(constrain_float(range_pos, 0, 0.75)); 
	_median_neg.update(constrain_float(range_neg, 0, 0.75)); 

	const float vfb[3] = {
		//_smooth_flow.get(),
		flow_vel, 
		_median_pos.get(), 
		_median_neg.get()
	}; 
	math::Vector<3> zk(vfb); 
	_zk = zk; 

	if(flow_quality > 0) {
		const float F[4][4] = {
			{0.0, 0.0, 0.5, -0.5}, 	// compute final center from average of back and front
			{0.0, 1.0, 0.0, 0.0}, 	// let velocity stay the same 
			{0.0, -dt, 1.0, 0.0}, 	// compute next front from integration of velocity and sensor reading
			{0.0, dt, 0.0, 1.0} 	// compute next back from integration of velocity and sensor reading
		}; 
		_kf.set_state_transition_matrix(math::Matrix<4,4>(F)); 
		const float H[3][4] = {
			{0.0, 1.0, 0.0, 0.0}, 	// velocity is x1
			{0.0, 0.0, 1.0, 0.0}, 	// front is x2
			{0.0, 0.0, 0.0, 1.0} 	// back is x3
		}; 
		_kf.set_state_input_matrix(math::Matrix<3, 4>(H)); 

		// when quality = 255 -> 0 noise, when quality 0 -> 0.4 noise. 
		// trust flow more if quality is better
		// trust rangefinders less the further distance they report
		float flow_noise = 0.0; //0.4 * constrain_float(1.0 - (flow_quality), 0, 1.0); 
		const float R[3][3] = {
			{flow_noise, 0.0, 0.0},
			{0.0, zk(1) * 2.0, 0.0},
			{0.0, 0.0, zk(2) * 2.0}
		}; 
		_kf.set_sensor_covariance_matrix(math::Matrix<3, 3>(R)); 
	} else {
		const float F[4][4] = {
			{0.0, 0.0, 0.5, -0.5},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		}; 
		_kf.set_state_transition_matrix(math::Matrix<4,4>(F)); 
		const float H[3][4] = {
			{0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		}; 
		_kf.set_state_input_matrix(math::Matrix<3, 4>(H)); 
		// trust rangefinders more if we have no velocity estimate
		const float R[3][3] = {
			{1.0, 0.0, 0.0},
			{0.0, zk(1) * 0.1, 0.0},
			{0.0, 0.0, zk(2) * 0.1}
		}; 
		_kf.set_sensor_covariance_matrix(math::Matrix<3, 3>(R)); 
	}

	// store current reading as last before updating
	math::Vector<4> prev = _kf.get_prediction(); 

	_kf.update(zk, math::Vector<4>()); 

	// calculate velocity of the drone from center point estimate
	math::Vector<4> p = _kf.get_prediction(); 
	if(!is_zero(dt)){
		_velocity = (p(0) - prev(0)) / dt; 
		//_median_velocity.update((p(0) - prev(0)) / dt); 
		//_lp_velocity.apply((p(0) - prev(0)) / dt, dt);  
		//_lp_velocity.apply(_median_velocity.update((p(0) - prev(0)) / dt), dt);  
	}
}

float RangerNav::PVPredictor::get_last_velocity_prediction(){
	return -_velocity; 
	//return -_median_velocity.get(); 
}

float RangerNav::PVPredictor::get_last_offset_prediction(){
	return _smooth_pos.update((_zk(1) - _zk(2)) / 2); 
	//return -_kf.get_prediction()(0); 
}

RangerNav::RangerNav(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro){
	_ahrs = ahrs; 
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_baro = baro; 
}

float RangerNav::calculate_altitude(float range_bottom, bool range_valid){
	if(range_valid && range_bottom < 0.75) {
		_altitude = _smooth_bottom.update(_median_bottom.update(range_bottom)); 
		_baro_zero_altitude = _baro->get_altitude() - _altitude; 
	} else {
		_altitude = _smooth_bottom.update(_baro->get_altitude() - _baro_zero_altitude); 
	}
	return _altitude; 
}

extern AP_HAL::DebugConsole _debug_console; 
void RangerNav::update(float dt){
	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;

	// read rangefinder
	_rangefinder->update(dt); 

	long long last_reading = _rangefinder->last_update_millis(); 
	if(_last_range_reading != last_reading){	
		float __attribute__((unused)) rdt = 0.04; 
		/*if(_last_range_reading != 0){
			rdt = (last_reading - _last_range_reading) * 0.001; 
		}*/

		float front, back, right, left, bottom, top; 

		_rangefinder->get_readings_m(&front, &back, &right, &left, &bottom, &top); 

		float altitude = calculate_altitude(bottom, _rangefinder->have_bottom()); 
		Vector2f vel = _optflow->flowRate() * altitude; 

		float flow_quality = (float)_optflow->quality() / 255.0; 

		front = 0.75; 
		back = 0.75; 
		right = 0.75; 
		left = 0.75;
		_pv_x.input(vel.x, flow_quality, front, back, rdt); 
		_pv_y.input(vel.y, flow_quality, right, left, rdt); 
	
		math::Vector<3> zk = _pv_x.get_last_input(); 	
		math::Vector<4> p = _pv_x.get_last_prediction(); 
		
		//hal.console->printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
		//	vel.x, front, back, _pv_x.get_last_velocity_prediction(), p(0), p(1), p(2), p(3), altitude); 

		_last_range_reading = last_reading; 
	}
	// update navigation filters
}

Vector3f RangerNav::get_velocity() {
	return Vector3f(
		_pv_x.get_last_velocity_prediction(), 
		_pv_y.get_last_velocity_prediction(), 
		0
	);
}

Vector3f RangerNav::get_position() {
	return Vector3f(
		_pv_x.get_last_offset_prediction(), 
		_pv_y.get_last_offset_prediction(),
		0
	);
}
