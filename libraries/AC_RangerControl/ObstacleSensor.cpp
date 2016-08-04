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

#include "ObstacleSensor.h"

ObstacleSensor::ObstacleSensor(const AP_AHRS &ahrs, const AP_RangeScanner_6DOF &rf_array): 
	_ahrs(ahrs),
	_rangefinders(rf_array),
	_safest_pos_offset(0, 0, 0){

}

bool ObstacleSensor::get_safest_position_offset(Vector3f &pos){
	if(!_valid_reading) return false; 
	Vector3f p; 
	_center_filter_x.get_center_point(p.x); 
	_center_filter_y.get_center_point(p.y); 
	pos = p; 
	return true; 
}

void ObstacleSensor::update(float dt){
	long long last_reading = _rangefinders.last_update_millis(); 
	if(_last_range_reading != last_reading){	
		float front, back, right, left, bottom, top; 
		_rangefinders.get_readings_m(&front, &back, &right, &left, &bottom, &top); 
	
		Vector3f vel; 
		if(!_ahrs.get_velocity_NED(vel)) {
			_valid_reading = false; 
			return; 
		}

		_center_filter_x.input_velocity(vel.x, 1.0f); 
		_center_filter_x.input_range(front, back); 
		_center_filter_y.input_velocity(vel.y, 1.0f);
		_center_filter_y.input_range(right, left); 
		
		_valid_reading = true; 

		_last_range_reading = last_reading; 
	}

	_center_filter_x.update(dt); 
	_center_filter_y.update(dt); 
}

