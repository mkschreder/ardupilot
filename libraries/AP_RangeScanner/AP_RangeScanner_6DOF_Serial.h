/* 
Serial rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#pragma once

#include <Filter/LowPassFilter.h>
#include <AP_HAL/AP_HAL.h>

#include <matrix/matrix/math.hpp>

#include "AP_RangeScanner_6DOF.h"

class AP_RangeScanner_6DOF_Serial : public AP_RangeScanner_6DOF {
public: 
    //AP_RangeScanner_6DOF_Serial(RangeScanner &ranger, uint8_t instance, RangeScanner::RangeScanner_State &_state);
	AP_RangeScanner_6DOF_Serial(AP_HAL::UARTDriver *uart){
		_port = uart; 
	}
	
	void init(); 
	void update(float dt); 

private: 
	AP_HAL::UARTDriver *_port; 

	char _buffer[64]; 
	unsigned int _buf_pos; 
	int32_t _prev_readings[6]; 
	unsigned long long _last_update; 
}; 

