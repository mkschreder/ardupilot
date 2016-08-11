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

#include "AC_RateControl.h"
#include <stdio.h>

AC_RateControl::AC_RateControl(){

}

void AC_RateControl::input_roll_rate(float rate){
	_target_rate.x = rate; 
}

void AC_RateControl::input_pitch_rate(float rate){
	_target_rate.y = rate; 
}

void AC_RateControl::input_yaw_rate(float rate){
	_target_rate.z = rate; 
}

void AC_RateControl::update(float dt){
	Vector3f err = _target_rate - _gyro_rate; 

	_pid_x.set_input_filter_all(err.x); 	
	_pid_y.set_input_filter_all(err.y); 	
	_pid_z.set_input_filter_all(err.z); 	

	// output to motors
	_out_roll = _pid_x.get_pid(); 
	_out_pitch = _pid_y.get_pid(); 
	_out_yaw = _pid_z.get_pid(); 
}

void AC_RateControl::input_measured_rates(const Vector3f &rates){
	_gyro_rate = rates; 
}

float AC_RateControl::get_motor_roll(void){
	return _out_roll; 
}

float AC_RateControl::get_motor_pitch(void){
	return _out_pitch; 
}

float AC_RateControl::get_motor_yaw(void){
	return _out_yaw; 
}


