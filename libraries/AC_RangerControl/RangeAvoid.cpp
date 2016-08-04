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

#include "RangeAvoid.h"

#define RANGE_MAX_RESPONSE 45.0
#define RANGE_MIN_CLEARANCE 80.0 
#define RANGE_AVOID_RESPONSE_SCALE 0.25


RangeAvoid::RangeAvoid(AP_AHRS *ahrs, RangerNav *nav, AP_InertialNav *inav):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_pitch_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0)
{
	_nav = nav;
	_inav = inav; 
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	_ahrs = ahrs; 
	//_flow_front_filt.set_cutoff_frequency(5.0); 
	//_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_vel_kP(float kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp); 
}

void RangeAvoid::set_vel_kI(float ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki); 
}

void RangeAvoid::set_vel_kD(float kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd); 
}

void RangeAvoid::set_center_kP(float kp){
	_pitch_center_pid.kP(kp); 
	_roll_center_pid.kP(kp); 
}

void RangeAvoid::set_center_kI(float ki){
	_pitch_center_pid.kI(ki); 
	_roll_center_pid.kI(ki); 
}

void RangeAvoid::set_center_kD(float kd){
	_pitch_center_pid.kD(kd); 
	_roll_center_pid.kD(kd); 
}
void RangeAvoid::input_desired_velocity_ms(float forward, float right){
	_desired_forward = forward; 
	_desired_right = right; 
}

const Vector2f RangeAvoid::get_filtered_flow() {
	return Vector2f(_flow_front_filtered, _flow_right_filtered); 
}

void RangeAvoid::reset(){
	_pitch_pid.reset(); 
	_roll_pid.reset(); 
	_pitch_center_pid.reset(); 
	_roll_center_pid.reset(); 
}

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;
extern AP_HAL::DebugConsole _debug_console; 

void RangeAvoid::update(float dt){
	if(is_zero(dt)) return; 

	_nav->update(dt); 

	//Vector3f p = _nav->get_position_ef(); 
	Vector3f p = _inav->get_position() * 0.01f; 

	float _yaw_mat[3][3] = {
		{ _ahrs->cos_yaw(), _ahrs->sin_yaw(), 0.0 },
        { -_ahrs->sin_yaw(), _ahrs->cos_yaw(), 0.0 },
		{ 0, 0, 1}
	}; 
	matrix::Matrix<float, 3, 3> yaw_mat((float*)_yaw_mat); 
	//matrix::Matrix<float, 3, 3> yaw_mat_inv = matrix::SquareMatrix<float, 3>(yaw_mat).inversed(); 
	matrix::Matrix<float, 3, 3> yaw_mat_inv = yaw_mat.inversed(); 

	matrix::Vector3<float> pos_ef(p.x, p.y, 0.0f);  // reset z

	// transform velocity from body frame to earth frame (we only use yaw)
	matrix::Vector<float, 3> input_ef = yaw_mat * matrix::Vector3<float>(_desired_forward, _desired_right, 0.0); 
	matrix::Vector<float, 3> out_ef; 
/*
	if(!_nav->have_position()){
		_target_pos_ef = pos_ef; 

		_pitch_center_pid.set_input_filter_all(constrain_float(input_ef(0), -2.0, 2.0)); 
		_roll_center_pid.set_input_filter_all(constrain_float(input_ef(1), -2.0, 2.0)); 

		_pitch_center_pid.reset_I(); 
		_roll_center_pid.reset_I(); 

		out_ef(0) = _pitch_center_pid.get_p(); 
		out_ef(1) = _roll_center_pid.get_p(); 
	} else {
	*/
		static bool reset_pitch = false, reset_roll = false; 

		// limit length of error to 2 meters. 
		matrix::Vector<float, 3> err = _target_pos_ef - pos_ef; 

		::printf("poserr: %f %f %f\n", err(0), err(1), err(2)); 
		// either if error is less than setpoint or if input pointing in opposite direction from error. 
		if(err.length() < 2.0 || (err.dot(input_ef)) < 0){
			_target_pos_ef += input_ef * dt * 2; 
		}  	

		// reset position instantly when pilot input goes to zero. 
		if(is_zero(_desired_forward)){
			if(reset_pitch){
				_target_pos_ef = pos_ef; 
				reset_pitch = false; 
			}
		} else {
			reset_pitch = true; 
		}
		if(is_zero(_desired_right)){
			if(reset_roll){
				_target_pos_ef = pos_ef; 
				reset_roll = false; 
			}
		} else {
			reset_roll = true; 
		}

		_target_pos_ef(2) = 0; 

		matrix::Vector<float, 3> err_ef = _target_pos_ef - pos_ef; 
		//Vector3f center = _nav->get_center_target(); 
		//target_pos.x += center.x * 4 * dt; 
		//target_pos.y += center.y * 4 * dt; 

		_pitch_center_pid.set_input_filter_all(constrain_float(err_ef(0), -2.0, 2.0)); 
		_roll_center_pid.set_input_filter_all(constrain_float(err_ef(1), -2.0, 2.0)); 

		out_ef(0) = _pitch_center_pid.get_pid();
		out_ef(1) = _roll_center_pid.get_pid(); 
	//}
	//_pitch_center_pid.set_input_filter_all(target_pos.x - pos.x); 
	//_roll_center_pid.set_input_filter_all(target_pos.y - pos.y); 

	//_pitch_pid.set_input_filter_all(desired_vel.x - vel.x); 
	//_pitch_pid.set_input_filter_all(_desired_forward - _pitch_center_pid.get_pid() - vel.x); 
	//_roll_pid.set_input_filter_all(_desired_right - _roll_center_pid.get_pid() - vel.y); 
	//_roll_pid.set_input_filter_all(desired_vel.y - vel.y); 

	matrix::Vector<float, 3> out = yaw_mat_inv * out_ef; 

	_output_pitch = -constrain_float(out(0), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
	_output_roll = constrain_float(out(1), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

	//_output_pitch = -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
	//_output_roll = constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

	//hal.console->printf("c(%f %f) out(%f %f)\n", (double)center.x, (double)center.y, (double)_output_pitch, (double)_output_roll); 
	//hal.console->printf("YAW: %f, OUT(%f %f)\n", _ahrs->yaw, _output_pitch, _output_roll); 
	//hal.console->printf("%f %f -> %f %f - e - %f %f -> %f %f\n", _desired_forward, _desired_right, pos(0), pos(1), target_pos(0) - pos(0), target_pos(1) - pos(1), _output_pitch, _output_roll);  

	//hal.console->printf("%f %f %f %f %f %f\n", _desired_forward, _desired_right, _pitch_center_pid.get_pid(), _roll_center_pid.get_pid(), _output_pitch, _output_roll); 
	//_debug_console.printf("%f, %f\n", (double)_output_pitch, (double)_output_roll); 
}

float RangeAvoid::get_desired_pitch_angle(){
	return _output_pitch;  
}

float RangeAvoid::get_desired_roll_angle(){
	return _output_roll;  
}
