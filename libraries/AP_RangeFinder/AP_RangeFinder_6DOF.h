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

#include <mathlib/mathlib.h>

class AP_RangeFinder_6DOF {
public: 
    //AP_RangeFinder_6DOF(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state);
	AP_RangeFinder_6DOF(AP_HAL::UARTDriver *uart){
		_port = uart; 
	}
	
	void init(); 
	void update(float dt); 

	// return true if corresponding sensor is available and healthy
	bool have_front() const; 
	bool have_back() const; 
	bool have_left() const; 
	bool have_right() const; 
	bool have_bottom() const; 
	bool have_top() const; 

	void get_rates_mps(float *front, float *back, float *right, float *left, float *bottom, float *top); 
	void get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top); 
private: 
	AP_HAL::UARTDriver *_port; 

	char _buffer[64]; 
	unsigned int _buf_pos; 
	unsigned long long _last_reading_time; 
	float _values[6]; 
	float _rates[6]; 
	int32_t _prev_readings[6]; 
}; 
