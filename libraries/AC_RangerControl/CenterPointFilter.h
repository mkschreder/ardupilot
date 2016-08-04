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
#include <AP_RangeScanner/AP_RangeScanner_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

#include <Filter/KalmanFilter.h>
#include <Filter/MedianFilter.h>
#include <Filter/MeanFilter.h>

// class that actually does the prediction for each axis
class CenterPointFilter {
public: 
	typedef enum {
		HAVE_VELOCITY = (1 << 0), 
		HAVE_RANGE_PLUS = (1 << 1), 
		HAVE_RANGE_MINUS = (1 << 2)
	} sensor_mask_t; 
	CenterPointFilter(); 

	// takes two rangefinder readings and velocity sensor readings into a prediction
	void input_velocity(float velocity, float vel_quality); 
	void input_range(float range_pos, float range_neg); 

	void update(float dt); 

	bool get_center_point(float &center); 

	matrix::Vector<float, 3> get_last_input(){ return _zk; }
	matrix::Vector<float, 4> get_last_prediction(){ return _kf.get_prediction(); }
protected: 
	MedianFilter<7> _median_pos, _median_neg;  
	MedianFilter<3> _median_flow, _median_velocity; 
	MeanFilter<3> _smooth_pos, _smooth_neg, _smooth_flow; 
	LowPassFilterFloat _lp_pos, _lp_neg; 
	LowPassFilterFloat _lp_velocity; 
	float _velocity; 
	KalmanFilter<3, 4> _kf; 
	matrix::Vector<float, 3> _zk; 

	float _input_vel; 
	float _input_vel_quality; 
}; 


