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

#define DOF_SENSOR_FRONT 1
#define DOF_SENSOR_BACK 0
#define DOF_SENSOR_RIGHT 3
#define DOF_SENSOR_LEFT 2
#define DOF_SENSOR_BOTTOM 4
#define DOF_SENSOR_TOP 5

#define DOF_SENSOR_NO_READING -1

class AP_RangeScanner_6DOF {
public: 
	virtual void init() = 0; 
	virtual void update(float dt) = 0; 

	void get_rates_mps(float *front, float *back, float *right, float *left, float *bottom, float *top) const ; 
	void get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top) const ; 

	bool have_front() const;  
	bool have_back() const;
	bool have_left() const; 
	bool have_right() const; 
	bool have_bottom() const; 
	bool have_top() const; 

	long long last_update_millis() const { return _last_reading_time; }
protected: 
	void _notify_new_reading(); 
	float _values[6]; 
	float _rates[6]; 
private: 
	unsigned long long _last_reading_time; 
}; 
