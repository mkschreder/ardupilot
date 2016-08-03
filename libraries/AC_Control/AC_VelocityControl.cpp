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

AC_VelocityControl::AC_VelocityControl( const AP_AHRS &ahrs, 
		const AP_InertialNav &inav,
		const AP_Vehicle::MultiCopter &aparm,
		AC_AngleControl& angle_control) :
	_ahrs(ahrs), 
	_inav(inav), 
	_angle_control(angle_control)
{

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

void AC_VelocityControl::input_heading(float angle){
	_angle_control.input_yaw_angle(angle); 
}

void AC_VelocityControl::input_yaw_rate(float angle){
	_angle_control.input_yaw_rate(angle); 
}

void AC_VelocityControl::input_throttle(float thr){
	_throttle = thr; 
}

void AC_VelocityControl::update(float dt){
	Vector3f vel;// = _inav.get_velocity() * 0.01f; 

	// use gps velocity if availabel and if not then we use inertial (accellerometer) velocity 
	if(!_ahrs.get_velocity_NED(vel))
		vel = _inav.get_velocity() * 0.01; 

	// we will rotate velocity into body frame
	Quaternion yaw_inv = Quaternion(cos(_ahrs.yaw / 2), 0, 0, sin(_ahrs.yaw / 2)).inversed(); 
	vel = yaw_inv * vel; 

	//vel.x = vel.x*_ahrs.cos_yaw() + vel.y*_ahrs.sin_yaw();
	//vel.y = -vel.x*_ahrs.sin_yaw() + vel.y*_ahrs.cos_yaw();

	Vector3f err = _target_vel - vel; 

	_pid_x.set_input_filter_all(err.x);	
	_pid_y.set_input_filter_all(err.y); 	
	_pid_z.set_input_filter_all(err.z); 	

	float out_roll = _pid_y.get_pid(); 
	float out_pitch = _pid_x.get_pid(); 
	float out_throttle = _pid_z.get_pid(); 

	out_roll = constrain_float(out_roll, -radians(45.0f), radians(45.0f)); 
	out_pitch = constrain_float(out_pitch, -radians(45.0f), radians(45.0f)); 
	out_throttle = -constrain_float(out_throttle, -1.0f, 1.0f) * 0.5f + 0.5f; 

	::printf("v(%f %f %f) verr: %f %f %f, out(%f %f %f)\n", vel.x, vel.y, vel.z, err.x, err.y, err.z, out_roll, out_pitch, out_throttle); 

	// output to rate controller
	_angle_control.input_roll_angle(out_roll); 
	_angle_control.input_pitch_angle(-out_pitch); 

	_angle_control.input_throttle(out_throttle); 
}

