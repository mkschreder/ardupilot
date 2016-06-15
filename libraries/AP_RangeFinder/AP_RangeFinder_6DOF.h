/* 
Serial rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#pragma once

#include <Filter/LowPassFilter.h>
#include <AP_HAL/AP_HAL.h>
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

class AP_RangeFinder_6DOF {
public: 
    //AP_RangeFinder_6DOF(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);
	AP_RangeFinder_6DOF(AP_HAL::UARTDriver *uart){
		_port = uart; 
	}
	
	void init(); 
	void update(float dt); 

	float get_front_clearance_cm();  
	float get_back_clearance_cm();  
	float get_right_clearance_cm();  
	float get_left_clearance_cm();  
	float get_top_clearance_cm();  
	float get_bottom_clearance_cm();  

	float get_raw_front(); 
	float get_raw_back(); 

	float get_velocity_forward(); 
	float get_velocity_right(); 
private: 
	AP_HAL::UARTDriver *_port; 

	char _buffer[64]; 
	unsigned int _buf_pos; 
	unsigned long long _last_reading_time; 
	int _readings[6]; 
	int _health[6]; 
	LowPassFilterFloat _filters[6]; 
	LowPassFilterFloat _vel_filters[6]; 
}; 
