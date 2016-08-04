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

enum {
	AC_CONTROL_X = 1 << 0, 
	AC_CONTROL_Y = 1 << 1, 
	AC_CONTROL_Z = 1 << 2
}; 

class AC_Controller {
public: 
	AC_Controller(); 

	void set_tuning(const Vector3f &roll, const Vector3f &pitch, const Vector3f &yaw); 

	void enable_control(int mask); 
	void disable_control(int mask); 

	virtual void update(float dt) = 0;  
protected: 
	AC_PID _pid_x, _pid_y, _pid_z; 
	int _flags; 
}; 
