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

#include <AC_Control/AC_Controller.h>
#include <AC_Control/AC_VelocityControl.h>
#include <AC_Control/AC_AngleControl.h>
#include <AC_Control/AC_RateControl.h>

#include <AP_RangeScanner/AP_RangeScanner_6DOF.h>

#include "ObstacleSensor.h"

class AC_RangerControl : public AC_Controller {
public: 
	AC_RangerControl( ); 
	void input_pilot_roll(float angle); 
	void input_pilot_pitch(float angle); 
	void input_pilot_yaw_rate(float angle); 
	void input_pilot_throttle(float thr); 

	void update(float dt); 
private: 
	void enter_state(int state); 
	void leave_state(int state); 
	Vector3f _pilot_input; 
	Vector3f _takeoff_position; 
	float _pilot_throttle, _throttle_rate; 
	int _state; 
}; 
