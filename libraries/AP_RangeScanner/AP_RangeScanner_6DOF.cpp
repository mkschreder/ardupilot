#include "AP_RangeScanner_6DOF.h"

void AP_RangeScanner_6DOF::get_rates_mps(float *front, float *back, float *right, float *left, float *bottom, float *top){
	*front = _rates[DOF_SENSOR_FRONT]; 
	*back = _rates[DOF_SENSOR_BACK]; 
	*left = _rates[DOF_SENSOR_LEFT]; 
	*right = _rates[DOF_SENSOR_RIGHT]; 
	*bottom = _rates[DOF_SENSOR_BOTTOM]; 
	*top = _rates[DOF_SENSOR_TOP]; 
}

void AP_RangeScanner_6DOF::get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top){
	*front = _values[DOF_SENSOR_FRONT]; 
	*back = _values[DOF_SENSOR_BACK]; 
	*left = _values[DOF_SENSOR_LEFT]; 
	*right = _values[DOF_SENSOR_RIGHT]; 
	*bottom = _values[DOF_SENSOR_BOTTOM]; 
	*top = _values[DOF_SENSOR_TOP]; 
}

bool AP_RangeScanner_6DOF::have_front() const { return !is_zero(_values[DOF_SENSOR_FRONT]); } 
bool AP_RangeScanner_6DOF::have_back() const { return !is_zero(_values[DOF_SENSOR_BACK]); } 
bool AP_RangeScanner_6DOF::have_left() const { return !is_zero(_values[DOF_SENSOR_LEFT]); } 
bool AP_RangeScanner_6DOF::have_right() const { return !is_zero(_values[DOF_SENSOR_RIGHT]); } 
bool AP_RangeScanner_6DOF::have_bottom() const { return !is_zero(_values[DOF_SENSOR_BOTTOM]); } 
bool AP_RangeScanner_6DOF::have_top() const { return !is_zero(_values[DOF_SENSOR_TOP]); } 

void AP_RangeScanner_6DOF::_notify_new_reading(){
	_last_reading_time = AP_HAL::millis(); 
}
