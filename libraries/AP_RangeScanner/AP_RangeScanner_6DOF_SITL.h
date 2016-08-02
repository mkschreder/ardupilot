/* 
SITL rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#pragma once

#include <SITL/SITL.h>
#include <Filter/LowPassFilter.h>
#include <AP_HAL/AP_HAL.h>

#include <matrix/matrix/math.hpp>

class AP_RangeScanner_6DOF_SITL : public AP_RangeScanner_6DOF {
public: 
    //AP_RangeScanner_6DOF_SITL(RangeScanner &ranger, uint8_t instance, RangeScanner::RangeScanner_State &_state);
	AP_RangeScanner_6DOF_SITL(); 

	void init(); 
	void update(float dt); 

	void get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top); 
private: 
	float _values[6]; 
	SITL::SITL *_sitl; 
}; 

