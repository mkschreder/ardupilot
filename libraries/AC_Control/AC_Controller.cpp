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

#include "AC_Controller.h"

AC_Controller::AC_Controller():
	_pid_x(1.0, 0.0, 0.0, 1.0, 1.0, 1.0f/400.0f),  
	_pid_y(1.0, 0.0, 0.0, 1.0, 1.0, 1.0f/400.0f),  
	_pid_z(1.0, 0.0, 0.0, 1.0, 1.0, 1.0f/400.0f){
	_flags = AC_CONTROL_X | AC_CONTROL_Y | AC_CONTROL_Z; 	
}

void AC_Controller::set_tuning(const Vector3f &xx, const Vector3f &yy, const Vector3f &zz){
	_pid_x.kP(xx.x); 
	_pid_x.kI(xx.y); 
	_pid_x.kD(xx.z); 
	_pid_y.kP(yy.x); 
	_pid_y.kI(yy.y); 
	_pid_y.kD(yy.z);
	_pid_z.kP(zz.x); 
	_pid_z.kI(zz.y); 
	_pid_z.kD(zz.z);
}

void AC_Controller::enable_control(int mask){
	_flags |= mask; 
}

void AC_Controller::disable_control(int mask){
	_flags &= ~mask; 
}

