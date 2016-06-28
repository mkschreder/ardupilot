#pragma once

#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder_6DOF.h>
#include <AP_OpticalFlow/OpticalFlow.h>

#include <mathlib/mathlib.h>

class RangeAvoid {
public: 
	class RangeFilter {
	public: 
		RangeFilter(); 	
		void update(float pos, float vel, float acc, float dt); 
		float get_center_offset() const;
		float get_velocity() const; 
	private: 
		// state transition matrix
		math::Matrix<3, 3> F; 
		// external forces matrix
		math::Matrix<3, 3> B;
		// input vector to state matrix 
		math::Matrix<3, 3> H;
		// kalman gain matrix
		math::Matrix<3, 3> K; 
		// prediction error matrix
		math::Matrix<3, 3> P;  
		// sensor noise
		math::Matrix<3, 3> R; 
		// process noise 
		math::Matrix<3, 3> Q; 
		
		// filter state 
		math::Vector<3> xk; 
	}; 

	RangeAvoid(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro); 
	
	void set_vel_kP(float kp); 
	void set_vel_kI(float ki); 
	void set_vel_kD(float kd); 
	
	void set_center_kP(float kp); 
	void set_center_kI(float ki); 
	void set_center_kD(float kd); 

	void input_desired_velocity_ms(float forward, float right); 

	void update(float dt); 

	float get_desired_pitch_angle(); 
	float get_desired_roll_angle(); 

	Vector3f get_center_offset(); 
	Vector3f get_velocity(); 

	const Vector2f get_filtered_flow(); 

	void reset(); 
private: 
	void update_flow_velocity(const Vector2f &flow, float altitude, float dt); 
	void update_ekf(float dt); 
	Vector2f get_wall_avoidance_velocity_compensation(); 

	RangeFilter xf, yf, zf; 

	AP_RangeFinder_6DOF *_rangefinder; 
	AP_AHRS *_ahrs; 
	AP_Baro *_baro; 
	AP_InertialSensor *_ins; 
	OpticalFlow *_optflow; 
	AC_PID _pitch_pid, _roll_pid; 
	AC_PID _pitch_center_pid, _roll_center_pid; 
	LowPassFilterFloat _flow_front_filt, _flow_right_filt; 
	float _pitchComp, _rollComp; 
	float _flow_front_filtered, _flow_right_filtered; 
	float _flow_distance_front, _flow_distance_right; 
	float _desired_forward, _desired_right; 
	float _output_pitch, _output_roll; 
	float _forward_response; 
	float _baro_zero_altitude; 
	Vector3f _velocity; 
}; 
