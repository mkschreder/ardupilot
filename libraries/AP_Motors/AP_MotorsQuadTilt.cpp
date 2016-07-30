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
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsQuadTilt.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsQuadTilt::var_info[] = {
	// @Param: TILT_SERVO_ON
    // @DisplayName: Turms the tilt servo on or off
    // @Description: If off, then servo is centered
    // @Range: 0 1
    // @Increment: 1

	AP_GROUPINFO("ON", 0, AP_MotorsQuadTilt, _servo_on, 1),

	// @Param: TILT_SERVO_CHANNEL
    // @DisplayName: Channel for the tilt servo
    // @Description: Tilt servo signal will go on this channel (use values starting from 1!)
    // @Range: 1 32
    // @Increment: 1

	AP_GROUPINFO("CHANNEL", 1, AP_MotorsQuadTilt, _servo_channel, AP_MOTORS_MAX_NUM_MOTORS), 

	// @Param: TILT_SERVO_TRAVEL
    // @DisplayName: Tilt servo travel in degrees in each direction
    // @Description: Specifies how far tilt servo is allowed to move
    // @Range: 0 90
    // @Increment: 1

	AP_GROUPINFO("TRAVEL", 2, AP_MotorsQuadTilt, _servo_travel, 45),

	AP_GROUPEND
}; 


// setup_motors - configures the motors for a quad
void AP_MotorsQuadTilt::setup_motors(){
    // call parent
    AP_MotorsQuad::setup_motors();
}

void AP_MotorsQuadTilt::output(){
	float servo_scale = (constrain_float(_servo_travel, 0, 90) / 90.0f); // 0.0 - 1.0. 
	uint16_t servo_pwm = constrain_int16(1500 + 500 * constrain_float(_tilt_pitch, -90, 90) / 90.0, 1000, 2000); 

	if(_servo_on && _servo_channel > 0){
		rc_write(_servo_channel - 1, servo_pwm); 
	} else {
		// center servo
		rc_write(_servo_channel - 1, 1500); 
	}

	AP_MotorsQuad::output(); 
}

void AP_MotorsQuadTilt::output_to_servos(){
	// no longer used
}
	
