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

#include "AC_RangerControl.h"
#include <stdio.h>

enum {
	RANGER_STATE_LANDED, 
	RANGER_STATE_TAKEOFF,
	RANGER_STATE_FLYING, 
	RANGER_STATE_AVOIDING
}; 

AC_RangerControl::AC_RangerControl( const AP_AHRS &ahrs,
					const AP_Vehicle::MultiCopter &aparm,
					AC_VelocityControl &vel_control,
					AC_AngleControl &angle_control, 
					AC_RateControl &rate_control, 
					AP_RangeScanner_6DOF &rangefinders):
		_ahrs(ahrs), 
		_velocity_control(vel_control), 
		_angle_control(angle_control),
		_rate_control(rate_control),
		_pilot_input(0, 0, 0),
		_throttle_rate(0),
		_obstacle_sensor(ahrs, rangefinders),
		_state(-1)
{
	enter_state(RANGER_STATE_LANDED); 
	//AP_Param::setup_object_defaults(this, var_info);
}

void AC_RangerControl::input_pilot_roll(float angle){
	_pilot_input.x = angle; 
}

void AC_RangerControl::input_pilot_pitch(float angle){
	_pilot_input.y = angle; 
}

void AC_RangerControl::input_pilot_yaw_rate(float rate){
	_pilot_input.z = rate; 
}

void AC_RangerControl::input_pilot_throttle(float thr){
	_pilot_throttle = thr; 

	// convert throttle into up/down rate
	thr -= 0.5; 
	if(thr > -0.1f && thr < 0.1f) thr = 0; 
	// constrain so that minimum rate is always above minimum
	_throttle_rate = constrain_float(thr, -0.4, 0.5); 
}

void AC_RangerControl::enter_state(int state){
	leave_state(_state); 
	switch(state){
		case RANGER_STATE_LANDED: {
			// if we are landed and throttle is below takeoff then we need to forward throttle directly to the angle controller
			_velocity_control.disable_control(AC_CONTROL_Z); 
		} break; 
		case RANGER_STATE_TAKEOFF: {
			_velocity_control.enable_control(AC_CONTROL_Z); 
		} break; 	
		case RANGER_STATE_FLYING: {
			_velocity_control.enable_control(AC_CONTROL_Z); 
		} break; 
	}
	_state = state; 
}

void AC_RangerControl::leave_state(int state){

}

void AC_RangerControl::update(float dt){
	::printf("state: %d\n", _state); 
	
	const float VELOCITY_SCALE = 10.0f; 

	float total_forward = _pilot_input.y * VELOCITY_SCALE;
	float total_right = _pilot_input.x * VELOCITY_SCALE; 

	_obstacle_sensor.update(dt); 

	switch(_state){
		case RANGER_STATE_LANDED: {
			_angle_control.input_throttle(_pilot_throttle); 
			if(_throttle_rate > 0) {
				Vector3f pos; 
				if(!_ahrs.get_relative_position_NED(pos)){
					enter_state(RANGER_STATE_FLYING); 
					break; 
				}
				_takeoff_position = pos; 
				enter_state(RANGER_STATE_TAKEOFF); 
				break; 
			}
			// TODO: test that we are above ground and switch into flying if throttle above hover
		} break; 
		case RANGER_STATE_TAKEOFF: {
			_velocity_control.input_z_velocity(_throttle_rate * 10.0); 
			Vector3f pos; 	
			if(!_ahrs.get_relative_position_NED(pos)){
				enter_state(RANGER_STATE_FLYING); 
				break; 
			}
			// if 30cm up in the air then we are flying
			if(pos.z - _takeoff_position.z > 0.3){
				enter_state(RANGER_STATE_FLYING); 
				break; 
			}
			// if throttle is below middle then we land
			if(_throttle_rate < 0.0f){
				enter_state(RANGER_STATE_LANDED); 
				break; 
			}
		} break; 	
		case RANGER_STATE_FLYING: {
			Vector3f off; 
			if(_obstacle_sensor.get_safest_position_offset(off)){
				::printf("Safest offset: %f %f %f\n", off.x, off.y, off.z); 
				total_forward += -off.x; 
				total_right += off.y; 
			} 

			
			_velocity_control.input_z_velocity(_throttle_rate * 4.0); 
			// TODO: test for range down to determine when landed

		} break; 
	}

	_velocity_control.input_x_velocity(total_forward); 
	_velocity_control.input_y_velocity(total_right); 

	_velocity_control.input_yaw_rate(_pilot_input.z); 
}

