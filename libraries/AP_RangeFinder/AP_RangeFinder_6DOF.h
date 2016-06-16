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

	// return true if corresponding sensor is available and healthy
	bool have_front(); 
	bool have_back(); 
	bool have_left(); 
	bool have_right(); 
	bool have_bottom(); 
	bool have_top(); 

	// returns true if sensor is able to provide position of a center point between it's readings
	bool have_center_point(); 

	// returns center point offset from current position based on sensor measurement, |0| otherwise 
	const Vector3f &get_center_point_offset(); 

	float get_velocity_forward(); 
	float get_velocity_right(); 
private: 
	AP_HAL::UARTDriver *_port; 

	char _buffer[64]; 
	unsigned int _buf_pos; 
	unsigned long long _last_reading_time; 
	int32_t _readings[6]; 
	int32_t _prev_readings[6]; 
	int32_t _health[6]; 
	Vector3f _center_point; // computed center point (if available)
	LowPassFilterFloat _filters[6]; 
	LowPassFilterFloat _vel_filters[6]; 
}; 
