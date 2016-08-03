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

#include <Filter/LowPassFilter.h>
#include <AP_HAL/AP_HAL.h>

#include <matrix/matrix/math.hpp>

#include "AP_RangeScanner_6DOF.h"

class AP_RangeScanner_6DOF_Serial : public AP_RangeScanner_6DOF {
public: 
	AP_RangeScanner_6DOF_Serial(AP_HAL::UARTDriver *uart); 

	void init(); 
	void update(float dt); 

private: 
	AP_HAL::UARTDriver *_port; 

	char _buffer[64]; 
	unsigned int _buf_pos; 
	int32_t _prev_readings[6]; 
	unsigned long long _last_update; 
}; 

