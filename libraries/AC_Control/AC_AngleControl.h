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

#include "AC_RateControl.h"

class AC_AngleControl : public AC_Controller {
public: 
	AC_AngleControl( ); 
	void input_roll_angle(float angle); 
	void input_pitch_angle(float angle); 
	void input_measured_angles(float roll, float pitch); 

	float get_desired_roll_rate(void); 
	float get_desired_pitch_rate(void); 

	void update(float dt); 
private: 
	float _sensor_roll, _sensor_pitch; 
	float _out_roll, _out_pitch; 
	float _target_roll, _target_pitch; 
}; 
