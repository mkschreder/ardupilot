/* 
Serial rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#include <stdlib.h>
#include <stdio.h>
#include "AP_RangeFinder_6DOF.h"

// TODO: make work for sitl
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

#define DOF_SENSOR_FRONT 1
#define DOF_SENSOR_BACK 0
#define DOF_SENSOR_RIGHT 3
#define DOF_SENSOR_LEFT 2
#define DOF_SENSOR_TOP 5
#define DOF_SENSOR_BOTTOM 4

#define DOF_SENSOR_MAX_RANGE 3
#define DOF_SENSOR_NO_READING -1

void AP_RangeFinder_6DOF::init(){
	_port->begin(57600); 
}

void AP_RangeFinder_6DOF::get_rates_mps(float *front, float *back, float *right, float *left, float *bottom, float *top){
	*front = _rates[DOF_SENSOR_FRONT]; 
	*back = _rates[DOF_SENSOR_BACK]; 
	*left = _rates[DOF_SENSOR_LEFT]; 
	*right = _rates[DOF_SENSOR_RIGHT]; 
	*bottom = _rates[DOF_SENSOR_BOTTOM]; 
	*top = _rates[DOF_SENSOR_TOP]; 
}

void AP_RangeFinder_6DOF::get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top){
	*front = _values[DOF_SENSOR_FRONT]; 
	*back = _values[DOF_SENSOR_BACK]; 
	*left = _values[DOF_SENSOR_LEFT]; 
	*right = _values[DOF_SENSOR_RIGHT]; 
	*bottom = _values[DOF_SENSOR_BOTTOM]; 
	*top = _values[DOF_SENSOR_TOP]; 
}

bool AP_RangeFinder_6DOF::have_front() const { return _prev_readings[DOF_SENSOR_FRONT] != DOF_SENSOR_NO_READING; } 
bool AP_RangeFinder_6DOF::have_back() const { return _prev_readings[DOF_SENSOR_BACK] != DOF_SENSOR_NO_READING; } 
bool AP_RangeFinder_6DOF::have_left() const { return _prev_readings[DOF_SENSOR_LEFT] != DOF_SENSOR_NO_READING; } 
bool AP_RangeFinder_6DOF::have_right() const { return _prev_readings[DOF_SENSOR_RIGHT] != DOF_SENSOR_NO_READING; } 
bool AP_RangeFinder_6DOF::have_bottom() const { return _prev_readings[DOF_SENSOR_BOTTOM] != DOF_SENSOR_NO_READING; } 
bool AP_RangeFinder_6DOF::have_top() const { return _prev_readings[DOF_SENSOR_TOP] != DOF_SENSOR_NO_READING; } 

void AP_RangeFinder_6DOF::update(float dt){
	int16_t b = _port->read(); 
	int max = 32; 
	while((b >> 8) == 0 && max--){
		char ch = b & 0xff; 
		_buffer[_buf_pos++] = ch; 
		
		if(_buf_pos >= sizeof(_buffer)){ 
			_buf_pos = 0; 
			memset(_buffer, 0, sizeof(_buffer)); 
			_buffer[_buf_pos++] = ch; 
		}
		else {
			if(ch == '\n') {
				_buffer[_buf_pos] = 0; 
				_buf_pos = 0; 
				if(_buffer[0] == 'R' && _buffer[1] == ' '){
					int tmp; 
					int readings[6] = { 0 }; 
					int count = sscanf(_buffer + 2, "%d %d %d %d %d %d %d", &tmp, &readings[0], &readings[1], &readings[2], &readings[3], &readings[4], &readings[5]); 
					if(count > 1 && count <= 7){
						// store previous readings for velocity calculation
						long long tnow = AP_HAL::millis(); 
						float d_t = (tnow - _last_reading_time) * 0.001; 
						if(_last_reading_time != 0 && !is_zero(d_t)){
							for(int c = 0; c < 6; c++){
								if(_prev_readings[c] != -1 && readings[c] != -1)
									_rates[c] = (((readings[c] - _prev_readings[c]) / 58.0f) * 0.01) / d_t; 
								else
									_rates[c] = 0; 
								// TODO: figure out what to return when there is no valid reading? 
								if(readings[c] == -1) _values[c] = DOF_SENSOR_MAX_RANGE; 
								else _values[c] = (readings[c] / 58.0) * 0.01; 
							}
						}
						_last_reading_time = tnow; 

						memcpy(_prev_readings, readings, sizeof(_prev_readings)); 
					}
				}
				break; 
			}
		}
		b = _port->read(); 
	}
}

#endif
