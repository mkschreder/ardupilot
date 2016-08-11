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

#pragma once 

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library

#include "AC_Controller.h"
#include "AC_VelocityControl.h"
#include "AC_AngleControl.h"

class AC_VelocityControl : public AC_Controller {
public: 
	AC_VelocityControl( ); 

	void input_x_velocity(float rate); 
	void input_y_velocity(float rate); 
	void input_z_velocity(float rate); 
	void input_measured_velocity_bf(const Vector3f &vel); 

	float get_desired_pitch_angle(void); 
	float get_desired_roll_angle(void); 
	float get_desired_throttle(void); 

	void update(float dt); 
private: 
	float _out_roll; 
	float _out_pitch; 
	float _out_throttle; 

	Vector3f _sensor_vel, _target_vel; 
	float _throttle; 
}; 
