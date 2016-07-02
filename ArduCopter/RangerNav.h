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

#include "KalmanFilter.h"
#include "MedianFilter.h"
#include "MeanFilter.h"

class RangerNav {
public: 
	RangerNav(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro); 

	void update(float dt); 	
	Vector3f get_velocity(); 
	Vector3f get_position(); 
private: 
	AP_RangeFinder_6DOF *_rangefinder; 
	AP_AHRS *_ahrs; 
	AP_Baro *_baro; 
	AP_InertialSensor *_ins; 
	OpticalFlow *_optflow; 
	
	MedianFilter<7> _median_front, _median_back;  
	MedianFilter<3> _median_flow, _median_velocity; 
	MeanFilter<3> _smooth_front, _smooth_back, _smooth_flow; 
	KalmanFilter<3, 4> _kf_range_x; 

	math::Vector<4> _p_prev; 
	Vector3f _velocity; 
	float _baro_zero_altitude; 
	long long _last_range_reading; 
}; 
