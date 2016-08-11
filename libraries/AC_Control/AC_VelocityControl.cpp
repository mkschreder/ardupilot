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

#include "AC_VelocityControl.h"
#include <matrix/matrix/Quaternion.hpp>

#include <stdio.h>

AC_VelocityControl::AC_VelocityControl( ){

}

void AC_VelocityControl::input_x_velocity(float vel){
	_target_vel.x = -vel; 
}

void AC_VelocityControl::input_y_velocity(float vel){
	_target_vel.y = vel; 
}

void AC_VelocityControl::input_z_velocity(float vel){
	// z is down so we negate the input
	_target_vel.z = -vel; 
}

float AC_VelocityControl::get_desired_pitch_angle(void){
	return _out_pitch; 
}

float AC_VelocityControl::get_desired_roll_angle(void){
	return _out_roll; 
}

float AC_VelocityControl::get_desired_throttle(void){
	return _out_throttle; 
}

void AC_VelocityControl::input_measured_velocity_bf(const Vector3f &vel){
	_sensor_vel = vel; 
}

void AC_VelocityControl::update(float dt){
	Vector3f err = _target_vel - _sensor_vel; 

	_pid_x.set_input_filter_all(err.x);	
	_pid_y.set_input_filter_all(err.y); 	
	_pid_z.set_input_filter_all(err.z); 	

	float roll = _pid_y.get_pid(); 
	float pitch = _pid_x.get_pid(); 
	float throttle = _pid_z.get_pid(); 

	_out_roll = constrain_float(roll, -radians(45.0f), radians(45.0f)); 
	_out_pitch = -constrain_float(pitch, -radians(45.0f), radians(45.0f)); 
	_out_throttle = -constrain_float(throttle, -1.0f, 1.0f) * 0.5f + 0.5f; 
}

