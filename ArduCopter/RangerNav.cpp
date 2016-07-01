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

RangerNav::RangerNav(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro){
	_ahrs = ahrs; 
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_baro = baro; 

	_kf_range_x.set_sensor_noise_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.2, 0.0, 0.0, 0.0},
		{0.0, 0.2, 0.0, 0.0},
		{0.0, 0.0, 0.02, 0.0},
		{0.0, 0.0, 0.0, 0.02}
	})); 

	_kf_range_x.set_process_noise_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.01, 0.0, 0.0, 0.0},
		{0.0, 0.01, 0.0, 0.0},
		{0.0, 0.0, 0.01, 0.0},
		{0.0, 0.0, 0.0, 0.01}
	})); 

	_kf_range_x.set_state_input_matrix(math::Matrix<4, 4>((const float[4][4]){
		{1.0, 1.0, 0.0, 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{0.0, -1.0, 1.0, 0.0},
		{0.0, 1.0, 0.0, 1.0}
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
	
	long long last_reading = _rangefinder->last_update_millis(); 
	if(_last_range_reading != last_reading){
		_rangefinder->get_readings_m(&front, &back, &right, &left, &bottom, &top); 
		_rangefinder->get_rates_mps(&dfront, &dback, &dright, &dleft, &dbottom, &dtop); 

		Vector2f flow_rate = _optflow->flowRate(); 
		float altitude = 0; 

		// update our barometer zero point while we have rangefinder, after that use only barometer altitude relative to the previously sensed ground. 
		// Note: this may not work if suddenly ground gets closer, but is still out of rangefinder range. Then flow velocity will be slightly off. 
		if(_rangefinder->have_bottom()) {
			altitude = bottom; 
			_baro_zero_altitude = _baro->get_altitude() - altitude; 
		} else {
			altitude = _baro->get_altitude() - _baro_zero_altitude; 
		}

		// limit intput to 0.75m 
		front = constrain_float(front, 0, 0.75); 
		back = constrain_float(back, 0, 0.75); 

		math::Vector<4> p = _kf_range_x.get_prediction(); 

		// update rangefinder filters
		const float vfb[4] = {
			// center point is recursively fed back as input to the filter for maximum smoothness
			p(0), 
			// velocity is inverted when fed into the filter because forward movement of the drone means decrease in distance to front wall
			-_smooth_flow.update(_median_flow.update(flow_rate.x * altitude)), 
			_smooth_front.update(_median_front.update(front)), 
			_smooth_back.update(_median_back.update(back))
		}; 
		math::Vector<4> fb(vfb); 

		float fmat[4][4] = {
			{0.0, 1.0, 0.5, -0.5},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		}; 

		_kf_range_x.set_state_transition_matrix(math::Matrix<4,4>(fmat)); 
		_kf_range_x.update(fb, math::Vector<4>()); 

		// calculate velocity of the drone from center point estimate
		const math::Vector<4> &p_new = _kf_range_x.get_prediction(); 
		if(_last_range_reading != 0){ // we have at least one previous reading
			// velocity of the signal is inverted to get velocity of the drone
			_velocity = -Vector3f((p_new(0) - p(0)) / ((last_reading - _last_range_reading) * 0.001), 0, 0); 
		}

		hal.console->printf("%f, %f, %f, %f, %f, %f, %f, %f\n",
			front, back, flow_rate.x * altitude, _median_velocity.update(_velocity.x), p(0), p(1), p(2), p(3)); 

		_last_range_reading = last_reading; 

	}
	// update navigation filters
}

Vector3f RangerNav::get_velocity() {
	return _velocity;
}

Vector3f RangerNav::get_position() {
	const math::Vector<4> &p = _kf_range_x.get_prediction(); 
	return Vector3f(-p(0), 0, 0);
}
