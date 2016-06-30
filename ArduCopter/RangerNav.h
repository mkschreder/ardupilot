#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

#include "KalmanFilter.h"

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

	KalmanFilter<4> kf_range_x; 
}; 
