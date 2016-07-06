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

#include "KalmanFilter.h"

RangeAvoid::RangeAvoid(RangerNav *nav):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_pitch_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0)
{
	_nav = nav;
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	//_flow_front_filt.set_cutoff_frequency(5.0); 
	//_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_vel_kP(float kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp * 0.7); 
}

void RangeAvoid::set_vel_kI(float ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki * 0.7); 
}

void RangeAvoid::set_vel_kD(float kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd * 0.7); 
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

	Vector3f vel = _nav->get_velocity(); 
	Vector3f pos = _nav->get_position(); 

	_pitch_center_pid.set_input_filter_all(_desired_forward - pos.x); 
	_roll_center_pid.set_input_filter_all(_desired_right - pos.y); 
/*
	Vector3f desired_vel = Vector3f(
		constrain_float(_pitch_center_pid.get_pid(), -1.0, 1.0),
		constrain_float(_roll_center_pid.get_pid(), -1.0, 1.0),
		0
	); */

	//_pitch_pid.set_input_filter_all(desired_vel.x - vel.x); 
	_pitch_pid.set_input_filter_all(_desired_forward - vel.x); 
	_roll_pid.set_input_filter_all(_desired_right - vel.y); 
	//_roll_pid.set_input_filter_all(desired_vel.y - vel.y); 

	_output_pitch = -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
	_output_roll = constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
	
	//_debug_console.printf("%f, %f\n", (double)_output_pitch, (double)_output_roll); 
}

float RangeAvoid::get_desired_pitch_angle(){
	return _output_pitch;  
}

float RangeAvoid::get_desired_roll_angle(){
	return _output_roll;  
}
