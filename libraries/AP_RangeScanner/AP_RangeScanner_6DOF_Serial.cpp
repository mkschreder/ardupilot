/* 
Serial rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#include <stdlib.h>
#include <stdio.h>

#include "AP_RangeScanner_6DOF_Serial.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

AP_RangeScanner_6DOF_Serial::AP_RangeScanner_6DOF_Serial(AP_HAL::UARTDriver *uart){
	_port = uart; 
	_last_update = 0; 
}

void AP_RangeScanner_6DOF_Serial::init(){
	hal.console->printf("Initializing 6DOF_Serial rangefinder\n"); 
	_port->begin(57600); 
}

void AP_RangeScanner_6DOF_Serial::update(float dt){
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
						float d_t = (tnow - _last_update) * 0.001; 
						if(_last_update != 0 && !is_zero(d_t)){
							for(int c = 0; c < 6; c++){
								if(_prev_readings[c] != -1 && readings[c] != -1)
									_rates[c] = (((readings[c] - _prev_readings[c]) / 58.0f) * 0.01) / d_t; 
								else
									_rates[c] = 0; 
								// TODO: figure out what to return when there is no valid reading? 
								if(readings[c] == -1) _values[c] = 0.0f; 
								else _values[c] = (readings[c] / 58.0) * 0.01; 
							}
						}
						memcpy(_prev_readings, readings, sizeof(_prev_readings)); 

						_last_update = tnow; 
						_notify_new_reading(); 
					}
				}
				break; 
			}
		}
		b = _port->read(); 
	}
}

