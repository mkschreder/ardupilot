/* 
Serial rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#include <stdlib.h>
#include <stdio.h>
#include "AP_RangeFinder_6DOF.h"

#define DOF_SENSOR_FRONT 1
#define DOF_SENSOR_BACK 0
#define DOF_SENSOR_RIGHT 3
#define DOF_SENSOR_LEFT 2
#define DOF_SENSOR_TOP 5
#define DOF_SENSOR_BOTTOM 4

#define DOF_SENSOR_NO_READING 200

/*
AP_RangeFinder_6DOF::AP_RangeFinder_6DOF(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state):
	AP_RangeFinder_Backend(_ranger, instance, _state){
	set_status(RangeFinder::RangeFinder_NoData); 
}

bool AP_RangeFinder_6DOF::detect(RangeFinder &_ranger, uint8_t instance){
	return true; 
}
*/

void AP_RangeFinder_6DOF::init(){
	_port->begin(57600); 
	for(int c = 0; c < 6; c++){
		_filters[c].set_cutoff_frequency(0.5f); 
		_vel_filters[c].set_cutoff_frequency(0.5f); 
	}
}

float AP_RangeFinder_6DOF::get_front_clearance_cm(){ return constrain_float(_filters[1].get(), 0, 200.0); } 
float AP_RangeFinder_6DOF::get_back_clearance_cm(){ return constrain_float(_filters[0].get(), 0, 200.0); } 
float AP_RangeFinder_6DOF::get_right_clearance_cm(){ return constrain_float(_filters[3].get(), 0, 200.0); } 
float AP_RangeFinder_6DOF::get_left_clearance_cm(){ return constrain_float(_filters[2].get(), 0, 200.0); } 
float AP_RangeFinder_6DOF::get_top_clearance_cm(){ return constrain_float(_filters[5].get(), 0, 200.0); } 
float AP_RangeFinder_6DOF::get_bottom_clearance_cm(){ return constrain_float(_filters[4].get(), 0, 200.0); } 

float AP_RangeFinder_6DOF::get_raw_front() { return _readings[DOF_SENSOR_FRONT] / 58.0f; }
float AP_RangeFinder_6DOF::get_raw_back() { return _readings[DOF_SENSOR_BACK] / 58.0f; }

float AP_RangeFinder_6DOF::get_velocity_y(){
	return _vel_filters[DOF_SENSOR_FRONT].get(); 
	if(_readings[DOF_SENSOR_FRONT] == DOF_SENSOR_NO_READING && _readings[DOF_SENSOR_BACK] == DOF_SENSOR_NO_READING) return 0; 
	else if(_readings[DOF_SENSOR_FRONT] == DOF_SENSOR_NO_READING) return _vel_filters[DOF_SENSOR_BACK].get(); 
	else if(_readings[DOF_SENSOR_BACK] == DOF_SENSOR_NO_READING) return _vel_filters[DOF_SENSOR_FRONT].get(); 
	return (_vel_filters[DOF_SENSOR_FRONT].get() - _vel_filters[DOF_SENSOR_BACK].get()) * 0.5; 
}

float AP_RangeFinder_6DOF::get_velocity_x(){
	if(_readings[DOF_SENSOR_LEFT] == DOF_SENSOR_NO_READING && _readings[DOF_SENSOR_RIGHT] == DOF_SENSOR_NO_READING) return 0; 
	else if(_readings[DOF_SENSOR_LEFT] == DOF_SENSOR_NO_READING) return _vel_filters[DOF_SENSOR_RIGHT].get(); 
	else if(_readings[DOF_SENSOR_RIGHT] == DOF_SENSOR_NO_READING) return _vel_filters[DOF_SENSOR_LEFT].get(); 
	return (_vel_filters[DOF_SENSOR_RIGHT].get() - _vel_filters[DOF_SENSOR_LEFT].get()) * 0.5; 
}

void AP_RangeFinder_6DOF::update(float dt){
	int16_t b = _port->read(); 
	while((b & 0xff00) == 0){
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
						memcpy(_readings, readings, 6 * sizeof(int)); 
					}
				}
				break; 
			}
		}
		b = _port->read(); 
	}
	
	// apply filters each time update is called
	for(int c = 0; c < 6; c++){
		float prev = _filters[c].get(); 
		float val = (float)_readings[c] / 58.0f; 
		if(val > 105.0) val = 105.0; 
		_filters[c].apply(val, dt); 
		float vel = (prev - _filters[c].get()) / dt; 
		//if(abs(vel) < 0.8) vel = 0; 
		_vel_filters[c].apply(vel, dt); 
	}
}
