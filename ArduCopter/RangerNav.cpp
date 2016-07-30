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

// TODO: make work for sitl
#if 0

#include "RangerNav.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

RangerNav::PVPredictor::PVPredictor(){
	_kf.set_state_covariance_matrix(math::Matrix<4, 4>((const float[4][4]){
		{0.01, 0.0, 0.0, 0.0},
		{0.0, 0.01, 0.0, 0.0},
		{0.0, 0.0, 0.001, 0.0},
		{0.0, 0.0, 0.0, 0.001}
	})); 
	_lp_velocity.set_cutoff_frequency(1.0); 
	_lp_pos.set_cutoff_frequency(0.5); 
	_lp_neg.set_cutoff_frequency(0.5); 
	_velocity = 0; 
}

void RangerNav::PVPredictor::input(float flow_vel, float flow_quality, float range_pos, float range_neg, float dt){
	//_smooth_flow.update(flow_vel); 
	_median_pos.update(range_pos); 
	_median_neg.update(range_neg); 

	const float vfb[3] = {
		//_smooth_flow.get(),
		flow_vel, 
		_lp_pos.apply(_median_pos.get(), dt), 
		_lp_neg.apply(_median_neg.get(), dt)
	}; 
	math::Vector<3> zk(vfb); 
	_zk = zk; 

	if(flow_quality > 0) {
		const float F[4][4] = {
			{0.0, 0.0, 1.0, -1.0}, 	// compute final center from average of back and front
			{0.0, 1.0, 0.0, 0.0}, 	// let velocity stay the same 
			{0.0, 0.0, 1.0, 0.0}, 	// compute next front from integration of velocity and sensor reading
			{0.0, 0.0, 0.0, 1.0} 	// compute next back from integration of velocity and sensor reading
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
			{0.0, 0.2, 0.0},
			{0.0, 0.0, 0.2}
		}; 
		_kf.set_sensor_covariance_matrix(math::Matrix<3, 3>(R)); 
	} else {
		const float F[4][4] = {
			{0.0, 0.0, 1.0, -1.0},
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
		//_velocity = _lp_velocity.apply(p(0) * 2 + ((p(0) - prev(0)) / dt), dt);  
		//_median_velocity.update((p(0) - prev(0)) / dt); 
		//_lp_velocity.apply((p(0) - prev(0)) / dt, dt);  
		//_lp_velocity.apply(_median_velocity.update((p(0) - prev(0)) / dt), dt);  
	}
}

float RangerNav::PVPredictor::get_last_velocity_prediction(){
	return -_velocity; 
}

float RangerNav::PVPredictor::get_last_offset_prediction(){
	//return _smooth_pos.update((_zk(1) - _zk(2)) / 2); 
	//return -_kf.get_prediction()(0); 
	return -_kf.get_prediction()(0); 
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
extern FILE *logfile; 

void RangerNav::update(float dt){
	// calculate acceleration in xy plane 
	//Vector3f accel = _ins->get_accel(); 
	//Vector3f gyro = _ins->get_gyro(); 
	//Vector3f mag = _ins->get_mag(); 
	//float accel_len = accel.length(); 
	//accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	//accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;

	/*
	// sample code to compute linear acceleration
	// Something seems off 
	math::Matrix<3, 3> R; 
	R.from_euler(-_ahrs->roll, _ahrs->pitch, _ahrs->yaw); 
	math::Vector<3> g(0, 0, -9.82); 
	math::Vector<3> a = R.inversed() * math::Vector<3>(accel.y, accel.x, accel.z); 
	hal.console->printf("RPY: %f %f %f, A: %f %f %f, ACC: %f %f %f\n", _ahrs->roll, _ahrs->pitch, _ahrs->yaw, a(0), a(1), a(2), accel.x, accel.y, accel.z); 
	*/

	// read rangefinder
	//_rangefinder->update(dt); 

	// TODO: move this 
	float _yaw_mat[3][3] = {
		{ _ahrs->cos_yaw(), _ahrs->sin_yaw(), 	0.0},
		{ -_ahrs->sin_yaw(), _ahrs->cos_yaw(), 	0.0},
		{ 0.0, 0.0, 1.0 }
	}; 
	math::Matrix<3, 3> yaw_mat(_yaw_mat); 

	Vector2f flow_rate = _optflow->flowRate(); 
	math::Vector<3> vel_ef = yaw_mat * math::Vector<3>(flow_rate.x, flow_rate.y, 0) * _altitude; 

	// update position integral and altitude
	_position += vel_ef * dt; 
	_position(2) = _altitude; 

	// recalculate other variables once rangefinder readings arrive
	long long last_reading = _rangefinder->last_update_millis(); 
	if(_last_range_reading != last_reading){	
		float __attribute__((unused)) rdt = 0.04; 
		/*if(_last_range_reading != 0){
			rdt = (last_reading - _last_range_reading) * 0.001; 
		}*/
		float front, back, right, left, bottom, top; 

		_rangefinder->get_readings_m(&front, &back, &right, &left, &bottom, &top); 

		float altitude = calculate_altitude(bottom, _rangefinder->have_bottom()); 
		float flow_quality = (float)_optflow->quality() / 255.0; 

		_pv_x.input(vel_ef(0), flow_quality, 
			constrain_float(front, 0, 0.75), 
			constrain_float(back, 0, 0.75), rdt); 
		_pv_y.input(vel_ef(1), flow_quality, 
			constrain_float(right, 0, 0.5), 
			constrain_float(left, 0, 0.5), rdt); 
	
		//math::Vector<3> zk = _pv_x.get_last_input(); 	
		//math::Vector<4> p = _pv_x.get_last_prediction(); 

		//hal.console->printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
		//		vel.x, front, back, _pv_x.get_last_velocity_prediction(), p(0), p(1), p(2), p(3), altitude); 
		
		_last_range_reading = last_reading; 
	}
	// update navigation filters
}

bool RangerNav::have_position(){
	return _optflow->quality() > 0; 
}

Vector3f RangerNav::get_velocity() {
	return Vector3f(
		_pv_x.get_last_velocity_prediction(), 
		_pv_y.get_last_velocity_prediction(), 
		0
	);
}

Vector3f RangerNav::get_center_target() {
	// center position relative of quad is opposite of quad offset from center (which the estimator currently estimates!)
	return Vector3f(
		-_pv_x.get_last_offset_prediction(), 
		-_pv_y.get_last_offset_prediction(),
		0
	);
}

Vector3f RangerNav::get_position_ef() {
	return Vector3f(_position(0), _position(1), _position(2)); 
	/*Vector3f(
		_pv_x.get_integrated_position(), 
		_pv_y.get_integrated_position(),
		0
	);*/
}
#endif
