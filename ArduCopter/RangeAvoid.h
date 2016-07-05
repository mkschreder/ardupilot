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

#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>
#include "RangerNav.h"

#include <mathlib/mathlib.h>

class RangeAvoid {
public: 
	RangeAvoid(RangerNav *_nav); 

	void set_vel_kP(float kp); 
	void set_vel_kI(float ki); 
	void set_vel_kD(float kd); 
	
	void set_center_kP(float kp); 
	void set_center_kI(float ki); 
	void set_center_kD(float kd); 

	void input_desired_velocity_ms(float forward, float right); 

	void update(float dt); 

	float get_desired_pitch_angle(); 
	float get_desired_roll_angle(); 

	Vector3f get_center_offset(); 
	Vector3f get_velocity(); 

	const Vector2f get_filtered_flow(); 

	void reset(); 
private: 
	void update_flow_velocity(const Vector2f &flow, float altitude, float dt); 
	Vector2f get_wall_avoidance_velocity_compensation(); 

	RangerNav *_nav; 

	AC_PID _pitch_pid, _roll_pid; 
	AC_PID _pitch_center_pid, _roll_center_pid; 
	float _pitchComp, _rollComp; 
	float _flow_front_filtered, _flow_right_filtered; 
	float _flow_distance_front, _flow_distance_right; 
	float _desired_forward, _desired_right; 
	float _output_pitch, _output_roll; 
	float _forward_response; 
	float _baro_zero_altitude; 
	Vector3f _velocity; 
}; 
