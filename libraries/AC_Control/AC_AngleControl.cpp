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

AC_AngleControl::AC_AngleControl( const AP_AHRS &ahrs,
					const AP_Vehicle::MultiCopter &aparm,
					AC_RateControl& rate_control) :
	_pid_x(1.2, 0.01, 0.01, 1.0, 1.0, 1.0f/400.0f),  
	_pid_y(1.0, 0.01, 0.01, 1.0, 1.0, 1.0f/400.0f),  
	_pid_z(1.0, 0, 0.01, 1.0, 1.0, 1.0f/400.0f),  
	_ahrs(ahrs), 
	_rate_control(rate_control),
	_yaw_rate(0)
{

}

void AC_AngleControl::input_roll_angle(float angle){
	_target_angle.x = angle; 
}

void AC_AngleControl::input_pitch_angle(float angle){
	_target_angle.y = angle; 
}

void AC_AngleControl::input_yaw_angle(float angle){
	_target_angle.z = angle; 
}

void AC_AngleControl::input_yaw_angle_rate(float rate){
	_yaw_rate = rate; 
}

void AC_AngleControl::input_throttle(float thr){
	_throttle = thr; 
}

void AC_AngleControl::update(float dt){
	// update yaw based on yaw rate
	_target_angle.z += _yaw_rate * dt; 
	
	// limit yaw to valid range of angles -180 to +180 deg (-pi to pi rads)
	//_target_angle.z = fmod(_target_angle.z, M_PI); 

	Vector3f err = _target_angle - Vector3f(_ahrs.roll, _ahrs.pitch, _ahrs.yaw); 

	::printf("yaw(%f %f)\n", _target_angle.z, _ahrs.yaw); 

	_pid_x.set_input_filter_all(err.x);	
	_pid_y.set_input_filter_all(err.y); 	
	_pid_z.set_input_filter_all(err.z); 	

	// output to rate controller
	_rate_control.input_roll_rate(_pid_x.get_pid()); 
	_rate_control.input_pitch_rate(_pid_y.get_pid()); 
	_rate_control.input_yaw_rate(_yaw_rate); //_pid_z.get_pid()); 

	_rate_control.input_throttle(_throttle); 
}

