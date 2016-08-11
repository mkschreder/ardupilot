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

#include "AC_AngleControl.h"
#include <stdio.h>

AC_AngleControl::AC_AngleControl( ){

}

void AC_AngleControl::input_roll_angle(float angle){
	_target_roll = angle; 
}

void AC_AngleControl::input_pitch_angle(float angle){
	_target_pitch = angle; 
}

void AC_AngleControl::input_measured_angles(float roll, float pitch){
	_sensor_roll = roll; 
	_sensor_pitch = pitch; 
}

float AC_AngleControl::get_desired_roll_rate(void){
	return _out_roll; 
}

float AC_AngleControl::get_desired_pitch_rate(void){
	return _out_pitch; 
}


void AC_AngleControl::update(float dt){
	_pid_x.set_input_filter_all(_target_roll - _sensor_roll);	
	_pid_y.set_input_filter_all(_target_pitch - _sensor_pitch); 	

	_out_roll = _pid_x.get_pid(); 
	_out_pitch = _pid_y.get_pid(); 
}

