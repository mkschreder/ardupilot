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

#include "RangeAvoid.h"

#define RANGE_MAX_RESPONSE 45.0
#define RANGE_MIN_CLEARANCE 80.0 
#define RANGE_AVOID_RESPONSE_SCALE 0.25

#include "KalmanFilter.h"

RangeAvoid::RangeAvoid(RangerNav *nav):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_pitch_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0)
{
	_nav = nav;
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	_flow_front_filt.set_cutoff_frequency(5.0); 
	_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_vel_kP(float kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp); 
}

void RangeAvoid::set_vel_kI(float ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki); 
}

void RangeAvoid::set_vel_kD(float kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd); 
}

void RangeAvoid::set_center_kP(float kp){
	_pitch_center_pid.kP(kp); 
	_roll_center_pid.kP(kp); 
}

void RangeAvoid::set_center_kI(float ki){
	_pitch_center_pid.kI(ki); 
	_roll_center_pid.kI(ki); 
}

void RangeAvoid::set_center_kD(float kd){
	_pitch_center_pid.kD(kd); 
	_roll_center_pid.kD(kd); 
}
void RangeAvoid::input_desired_velocity_ms(float forward, float right){
	_desired_forward = forward; 
	_desired_right = right; 
}

const Vector2f RangeAvoid::get_filtered_flow() {
	return Vector2f(_flow_front_filtered, _flow_right_filtered); 
}

void RangeAvoid::reset(){
	_pitch_pid.reset(); 
	_roll_pid.reset(); 
	_pitch_center_pid.reset(); 
	_roll_center_pid.reset(); 
}

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/*
void RangeAvoid::update_flow_velocity(const Vector2f &flow_rate, float altitude, float dt){
	float flow_forward = flow_rate.x * altitude; 
	float flow_right = flow_rate.y * altitude; 
	
	_flow_front_filt.apply(flow_forward, dt); 
	_flow_right_filt.apply(flow_right, dt); 

	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;
	
	// fuse the acceleration and flow into a velocity using a complementary filter
	float w = (0.075 / (0.075 + dt)); 
	_flow_front_filtered = w * (_flow_front_filtered + accel.x * dt) + (1.0 - w) * flow_forward;  
	_flow_right_filtered = w * (_flow_right_filtered + accel.y * dt) + (1.0 - w) * flow_right;  

	// integrate flow velocity into a distance traveled (right now just a test)
	_flow_distance_front += _flow_front_filtered * dt; 
	_flow_distance_right += _flow_right_filtered * dt; 
}

Vector3f RangeAvoid::get_center_offset(){
	return Vector3f(xf.get_center_offset(), 0, 0); 
}

Vector3f RangeAvoid::get_velocity(){
	return _velocity; 
}

Vector2f RangeAvoid::get_wall_avoidance_velocity_compensation(){
	float clear_front = _rangefinder->get_front_clearance_cm(); 
	float clear_back = _rangefinder->get_back_clearance_cm(); 
	float clear_right = _rangefinder->get_right_clearance_cm(); 
	float clear_left = _rangefinder->get_left_clearance_cm(); 

	// try to avoid object by going in opposite direction
	Vector2f directions[4] = {Vector2f(clear_front, 0), Vector2f(-clear_back, 0), Vector2f(0, -clear_left), Vector2f(0, clear_right)};
	Vector2f closest_point(1000, 1000); 
	bool found = false; 
	for(int c = 0; c < 4; c++){
		float len = directions[c].length(); 
		if((len < closest_point.length()) && (len < RANGE_MIN_CLEARANCE)) {
			closest_point = directions[c]; 
			found = true; 
		}
	}

	// get the response vector which is same as closest point but with inverted length and pointing in the other direction
	if(found){
		float len = closest_point.length() / RANGE_MIN_CLEARANCE; // maximum length is min_clearance 
		closest_point.normalize(); // make it a unit vector pointing in the direction of the point.  
		closest_point *= (1.0 - len) * RANGE_AVOID_RESPONSE_SCALE; // scale it to the inverted length of the original vector.  
		return Vector2f(-constrain_float(closest_point.x, -0.5, 0.5), -constrain_float(closest_point.y, -0.5, 0.5));  
	}

	return Vector2f(0, 0); 
}
*/
void RangeAvoid::update(float dt){
	if(is_zero(dt)) return; 

	_nav->update(dt); 

	Vector3f vel = _nav->get_velocity(); 
	Vector3f pos = _nav->get_position(); 

	_pitch_center_pid.set_input_filter_all(_desired_forward - pos.x); 

	float forward_vel = constrain_float(_pitch_center_pid.get_pid(), -1.0, 1.0); 

	_pitch_pid.set_input_filter_all(forward_vel - vel.x); 
	_roll_pid.set_input_filter_all(_desired_right - vel.y); 

	_output_pitch = -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
	_output_roll = constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);


	// only use flow if we have flow and at least baro or rangefinder altitude
	/*if( 0 ) { //_optflow->healthy() && (_rangefinder->have_bottom() || _baro->healthy())) {
		// get the flow from the flow sensor
		Vector2f flow_rate = _optflow->flowRate(); 
		float altitude = 0; 

		// update our barometer zero point while we have rangefinder, after that use only barometer altitude relative to the previously sensed ground. 
		// Note: this may not work if suddenly ground gets closer, but is still out of rangefinder range. Then flow velocity will be slightly off. 
		if(_rangefinder->have_bottom()) {
			altitude = _rangefinder->get_bottom_clearance_cm() * 0.01; 
			_baro_zero_altitude = _baro->get_altitude() - altitude; 
		} else {
			altitude = _baro->get_altitude() - _baro_zero_altitude; 
		}

		update_flow_velocity(flow_rate, altitude, dt); 

		// add velocity based wall avoidance to the pilot inputs
		//Vector2f wall_avoid = get_wall_avoidance_velocity_compensation(); 
		//_desired_forward += wall_avoid.x; 
		//_desired_right += wall_avoid.y; 

		_pitch_pid.set_input_filter_all(_desired_forward - _flow_front_filtered); 
		_roll_pid.set_input_filter_all(_desired_right - _flow_right_filtered); 

		_output_pitch = -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
		_output_roll = constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

		_pitch_center_pid.reset(); 
		_roll_center_pid.reset(); 

		hal.console->printf("WALL_AVOID OUT[ %f %f ], AVOID[ %f %f ], ALT[ %f ]\n", 
			(double)_output_pitch, (double)_output_roll, 
			(double)wall_avoid.x, (double)wall_avoid.y, 
			(double)altitude);   
		
	} else { //if(_rangefinder->have_center_point()) {
		// if we are flying and we have a center point offset but no velocity measurements from optical flow then we try to center the copter between the four walls	
		Vector3f center = Vector3f(_velocity.x, 0, 0); //_rangefinder->get_center_point_offset() * 0.01; 
	
		center.x = constrain_float(center.x, -0.5, 0.5); 
		center.y = constrain_float(center.y, -0.5, 0.5); 

		_pitch_center_pid.set_input_filter_all(_desired_forward + center.x); 
		_roll_center_pid.set_input_filter_all(_desired_right + center.y); 

		_output_pitch = -constrain_float(_pitch_center_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
		_output_roll = constrain_float(_roll_center_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

		hal.console->printf("CENTER OUT[ %f %f ], OFF[ %f %f ]\n", 
			(double)_output_pitch, (double)_output_roll, 
			(double)center.x, (double)center.y);   
	}  else {
		// otherwise just use pilot inputs scaled by P
		_output_pitch = -_desired_forward * _pitch_pid.kP(); 
		_output_roll = _desired_right * _roll_pid.kP(); 

		// reset pid controller integrals as well
		_pitch_pid.reset(); 
		_roll_pid.reset();

		_pitch_center_pid.reset(); 
		_roll_center_pid.reset(); 

		//hal.console->printf("NO_VEL OUT[ %f %f ]\n", 
	//		(double)_output_pitch, (double)_output_roll);   
	}*/

	//hal.console->printf("%f %f %f %f %f %f %f %f %f %f %f\n", clear_front, clear_back, _flow_distance_front, _flow_distance_right, clear_bottom, desired_forward, desired_right, 
	//		_flow_front_filtered, _flow_right_filtered, _rangefinder->get_velocity_forward() / 100.0, _rangefinder->get_velocity_right() / 100.0); 
	//, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 
	//hal.console->printf("%f %f %f %f %f %f %f %f %f\n", flow_rate.x, flow_rate.y, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 
}

float RangeAvoid::get_desired_pitch_angle(){
	return _output_pitch;  
}

float RangeAvoid::get_desired_roll_angle(){
	return _output_roll;  
}
