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

/* 
	This class implements a filter for filtering information from rangefinder array. 
*/ 

#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeScanner/AP_RangeScanner_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

#include <Filter/KalmanFilter.h>
#include <Filter/MedianFilter.h>
#include <Filter/MeanFilter.h>

#include "CenterPointFilter.h"

class ObstacleSensor {
public: 
	ObstacleSensor(const AP_AHRS &ahrs, const AP_RangeScanner_6DOF &rf_array); 

	bool get_safest_position_offset(Vector3f &pos); 
	void update(float dt); 
private:
	const AP_AHRS &_ahrs; 
	const AP_RangeScanner_6DOF &_rangefinders; 
	Vector3f _safest_pos_offset; 
	CenterPointFilter _center_filter_x;
	CenterPointFilter _center_filter_y;
	long long _last_range_reading; 
	bool _valid_reading;
}; 
