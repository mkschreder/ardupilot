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

AC_RateControl::AC_RateControl( const AP_AHRS &ahrs,
					const AP_Vehicle::MultiCopter &aparm,
					AP_Motors& motors) :
		_pid_x(0.8, 0.01, 0.005, 1.0, 1.0, 1.0f/400.0f),  
		_pid_y(1.0, 0.01, 0.01, 1.0, 1.0, 1.0f/400.0f),  
		_pid_z(1.0, 0, 0.01, 1.0, 1.0, 1.0f/400.0f),  
		_ahrs(ahrs), 
        _motors(motors)
{
	//AP_Param::setup_object_defaults(this, var_info);
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

void AC_RateControl::input_throttle(float thr){
	_throttle = thr; 
}

void AC_RateControl::update(float dt){
	Vector3f gyro = _ahrs.get_gyro(); 

	Vector3f err = _target_rate - gyro; 

	_pid_x.set_input_filter_all(err.x); 	
	_pid_y.set_input_filter_all(err.y); 	
	_pid_z.set_input_filter_all(err.z); 	

	// output to motors
	_motors.set_roll(_pid_x.get_pid()); 
	_motors.set_pitch(_pid_y.get_pid()); 
	_motors.set_yaw(_pid_z.get_pid()); 

	_motors.set_throttle(_throttle); 
}

