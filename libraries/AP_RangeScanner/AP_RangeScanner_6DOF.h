#pragma once

class AP_RangeScanner_6DOF {
public: 
	virtual void init(); 
	virtual void update(float dt); 

	void get_rates_mps(float *front, float *back, float *right, float *left, float *bottom, float *top); 
	void get_readings_m(float *front, float *back, float *right, float *left, float *bottom, float *top); 

	bool have_front() const;  
	bool have_back() const;
	bool have_left() const; 
	bool have_right() const; 
	bool have_bottom() const; 
	bool have_top() const; 

	long long last_update_millis(){ return _last_reading_time; }
protected: 
	void _notify_new_reading(); 

private: 
	float _values[6]; 
	float _rates[6]; 
	unsigned long long _last_reading_time; 
}; 
