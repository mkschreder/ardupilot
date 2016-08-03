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

#include "KalmanFilter.h"
#include "MedianFilter.h"
#include "MeanFilter.h"

class RangerNav {
public: 
	RangerNav(AP_AHRS *ahrs, AP_RangeScanner_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro); 

	void update(float dt); 	
	Vector3f get_velocity(); 
	Vector3f get_position_ef(); 
	Vector3f get_center_target(); 

	bool have_position(); 
private: 
	AP_RangeScanner_6DOF *_rangefinder; 
	AP_AHRS *_ahrs; 
	AP_Baro *_baro; 
	AP_InertialSensor *_ins; 
	OpticalFlow *_optflow; 

	// class that actually does the prediction for each axis
	class PVPredictor {
		friend class RangerNav; 
		typedef enum {
			HAVE_VELOCITY = (1 << 0), 
			HAVE_RANGE_PLUS = (1 << 1), 
			HAVE_RANGE_MINUS = (1 << 2)
		} sensor_mask_t; 
		PVPredictor(); 
		// takes two rangefinder readings and velocity sensor readings into a prediction
		void input(float velocity, float vel_quality, float range_pos, float range_neg, float dt); 	
		float get_last_velocity_prediction(); 
		float get_last_offset_prediction(); 

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
	}; 

	PVPredictor _pv_x, _pv_y; 
/*
	MedianFilter<7> _median_front, _median_back;  
	MedianFilter<3> _median_flow, _median_velocity; 
	MeanFilter<3> _smooth_front, _smooth_back, _smooth_flow; 
	KalmanFilter<3, 4> _kf_range_x; 

	matrix::Vector<4> _p_prev; 
	LowPassFilterFloat _vel_x, _vel_y; 
	*/
	float calculate_altitude(float range_bottom, bool range_valid); 
	Vector2f calculate_flow_ground_speed(Vector2f flow, float altitude); 

	Vector2f _flow_ground_speed; 
	MedianFilter<7> _median_bottom, _median_flow; 
	MeanFilter<3> _smooth_bottom, _smooth_flow; 
	matrix::Vector<float, 3> _position; 

	float _altitude; 
	float _baro_zero_altitude; 
	long long _last_range_reading; 
}; 
