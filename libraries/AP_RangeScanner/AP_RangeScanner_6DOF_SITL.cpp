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

#include <stdlib.h>
#include <stdio.h>

#include "AP_RangeScanner_6DOF.h"
#include "AP_RangeScanner_6DOF_SITL.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

AP_RangeScanner_6DOF_SITL::AP_RangeScanner_6DOF_SITL(){

}

void AP_RangeScanner_6DOF_SITL::init(){
	hal.console->printf("Initializing 6DOF SITL rangefinder\n"); 

	_sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (_sitl == nullptr) {
		hal.console->printf("Failed to find SITL object!\n"); 
    }
}

void AP_RangeScanner_6DOF_SITL::update(float dt){
	memcpy(_values, _sitl->state.scan6dof, sizeof(_values)); 	
	_notify_new_reading(); 
}

