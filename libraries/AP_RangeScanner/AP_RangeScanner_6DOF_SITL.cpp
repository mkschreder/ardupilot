/* 
SITL rangefinder with 6 finders, 2 on each axis

Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>, All Rights Reserved 

License: GPLv3
*/

#include <stdlib.h>
#include <stdio.h>
#include "AP_RangeScanner_6DOF_SITL.h"

#define DOF_SENSOR_FRONT 1
#define DOF_SENSOR_BACK 0
#define DOF_SENSOR_RIGHT 3
#define DOF_SENSOR_LEFT 2
#define DOF_SENSOR_BOTTOM 4
#define DOF_SENSOR_TOP 5

#define DOF_SENSOR_MAX_RANGE 3
#define DOF_SENSOR_NO_READING -1

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

AP_RangeScanner_6DOF_SITL::AP_RangeScanner_6DOF_SITL(){

}

void AP_RangeScanner_6DOF_SITL::init(){
	hal.console->printf("Initializing 6DOF SITL rangefinder\n"); 

	_sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (_sitl == nullptr) {
        return false;
    }
}

void AP_RangeScanner_6DOF_SITL::update(float dt){
	memcpy(_values, sitl->state.scan6dof, sizeof(_values)); 	
	_notify_new_reading(); 
}

