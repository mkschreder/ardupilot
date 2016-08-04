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

#include "CenterPointFilter.h"

CenterPointFilter::CenterPointFilter(){
	_kf.set_state_covariance_matrix(matrix::Matrix<float, 4, 4>((const float[4][4]){
		{0.01, 0.0, 0.0, 0.0},
		{0.0, 0.01, 0.0, 0.0},
		{0.0, 0.0, 0.001, 0.0},
		{0.0, 0.0, 0.0, 0.001}
	})); 
	_lp_velocity.set_cutoff_frequency(1.0); 
	_lp_pos.set_cutoff_frequency(0.5); 
	_lp_neg.set_cutoff_frequency(0.5); 
	_velocity = 0; 
	_input_vel = 0; 
	_input_vel_quality = 0; 
}

void CenterPointFilter::input_velocity(float vel, float vel_quality){
	_input_vel = vel; 
	_input_vel_quality = vel_quality; 
}

void CenterPointFilter::input_range(float range_pos, float range_neg){
	_median_pos.update(range_pos); 
	_median_neg.update(range_neg); 
}

void CenterPointFilter::update(float dt){
	const float vfb[3] = {
		//_smooth_flow.get(),
		_input_vel, 
		_lp_pos.apply(_median_pos.get(), dt), 
		_lp_neg.apply(_median_neg.get(), dt)
	}; 
	matrix::Vector<float, 3> zk(vfb); 
	_zk = zk; 

	if(_input_vel_quality > 0) {
		const float F[4][4] = {
			{0.0, 0.0, 1.0, -1.0}, 	// compute final center from average of back and front
			{0.0, 1.0, 0.0, 0.0}, 	// let velocity stay the same 
			{0.0, 0.0, 1.0, 0.0}, 	// compute next front from integration of velocity and sensor reading
			{0.0, 0.0, 0.0, 1.0} 	// compute next back from integration of velocity and sensor reading
		}; 
		_kf.set_state_transition_matrix(matrix::Matrix<float, 4,4>(F)); 
		const float H[3][4] = {
			{0.0, 1.0, 0.0, 0.0}, 	// velocity is x1
			{0.0, 0.0, 1.0, 0.0}, 	// front is x2
			{0.0, 0.0, 0.0, 1.0} 	// back is x3
		}; 
		_kf.set_state_input_matrix(matrix::Matrix<float, 3, 4>(H)); 

		// when quality = 255 -> 0 noise, when quality 0 -> 0.4 noise. 
		// trust flow more if quality is better
		// trust rangefinders less the further distance they report
		float flow_noise = 0.0; //0.4 * constrain_float(1.0 - (flow_quality), 0, 1.0); 
		const float R[3][3] = {
			{flow_noise, 0.0, 0.0},
			{0.0, 0.2, 0.0},
			{0.0, 0.0, 0.2}
		}; 
		_kf.set_sensor_covariance_matrix(matrix::Matrix<float, 3, 3>(R)); 
	} else {
		const float F[4][4] = {
			{0.0, 0.0, 1.0, -1.0},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		}; 
		_kf.set_state_transition_matrix(matrix::Matrix<float, 4,4>(F)); 
		const float H[3][4] = {
			{0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0},
			{0.0, 0.0, 0.0, 1.0}
		}; 
		_kf.set_state_input_matrix(matrix::Matrix<float, 3, 4>(H)); 
		// trust rangefinders more if we have no velocity estimate
		const float R[3][3] = {
			{1.0, 0.0, 0.0},
			{0.0, zk(1) * 0.1f, 0.0},
			{0.0, 0.0, zk(2) * 0.1f}
		}; 
		_kf.set_sensor_covariance_matrix(matrix::Matrix<float, 3, 3>(R)); 
	}

	// store current reading as last before updating
	matrix::Vector<float, 4> prev = _kf.get_prediction(); 

	_kf.update(zk, matrix::Vector<float, 4>()); 

	// calculate velocity of the drone from center point estimate
	matrix::Vector<float, 4> p = _kf.get_prediction(); 
	if(!is_zero(dt)){
		_velocity = (p(0) - prev(0)) / dt; 
		//_velocity = _lp_velocity.apply(p(0) * 2 + ((p(0) - prev(0)) / dt), dt);  
		//_median_velocity.update((p(0) - prev(0)) / dt); 
		//_lp_velocity.apply((p(0) - prev(0)) / dt, dt);  
		//_lp_velocity.apply(_median_velocity.update((p(0) - prev(0)) / dt), dt);  
	}
}

bool CenterPointFilter::get_center_point(float &center){
	// TODO: see if there is any case when center point can not be calculated and return false then
	center = _kf.get_prediction()(0); 
	return true; 
}


