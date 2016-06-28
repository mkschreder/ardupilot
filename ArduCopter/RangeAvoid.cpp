#include "RangeAvoid.h"

#define RANGE_MAX_RESPONSE 45.0
#define RANGE_MIN_CLEARANCE 80.0 
#define RANGE_AVOID_RESPONSE_SCALE 0.25

#include "KalmanFilter.h"

/*
class SimpleFilter {
public: 
	SimpleFilter(); 
	void update(float x, float dt); 
	float get(); 
private: 
	// state transition matrix
	math::Matrix<1, 1> F; 
	// external forces matrix
	math::Matrix<1, 1> B;
	// input vector to state matrix 
	math::Matrix<1, 1> H;
	// kalman gain matrix
	math::Matrix<1, 1> K; 
	// prediction error matrix
	math::Matrix<1, 1> P;  
	// sensor noise
	math::Matrix<1, 1> R; 
	// process noise 
	math::Matrix<1, 1> Q; 
	
	// filter state 
	math::Vector<1> xk; 
}; 

SimpleFilter::SimpleFilter(){
	// user input weight matrix
	B = math::Matrix<1, 1>((const float[1][1]){
		{1.0} 
	}); 
	// input vector covariance matrix 
	H = math::Matrix<1, 1>((const float[1][1]){
		{1.0}
	}); 
	// gain matrix (computed at each frame)
	K = math::Matrix<1, 1>((const float[1][1]){
		{1.0}
	}); 
	// prediction error matrix
	P = math::Matrix<1, 1>((const float[1][1]){
		{1.0}
	}); 
	// sensor noise
	R = math::Matrix<1, 1>((const float[1][1]){
		{0.8}
	}); 
	// process noise 
	Q = math::Matrix<1, 1>((const float[1][1]){
		{0.0}
	}); 

	xk = math::Vector<1>((const float[1]){0.0}); 
}

void SimpleFilter::update(float x, float dt){
	math::Vector<1> zk;
	zk(0) = x; 
	math::Vector<1> uk; 

	// update state transition matrix
	float F_mat[1][1] = {
		{1.0}
	}; 
	F = math::Matrix<1, 1>(F_mat); 

	// predict
	xk = F * xk + B * uk; 
	P = F * P * F.transposed() + Q; 

	// observe
	math::Vector<1> innovation = zk - H * xk; 
	math::Matrix<1, 1> innovation_cov = H * P * H.transposed() + R; 

	// update
	K = P * H.transposed() * innovation_cov.inversed(); 
	xk = xk + K * innovation; 
	P = P - K * H * P; 
}

float SimpleFilter::get(){
	return xk(0); 
}
*/
RangeAvoid::RangeAvoid(AP_AHRS *ahrs, AP_RangeFinder_6DOF *rangefinder, AP_InertialSensor *ins, OpticalFlow *optflow, AP_Baro *baro):
	_pitch_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_pitch_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0),
	_roll_center_pid(0, 0, 0, RANGE_MAX_RESPONSE * 0.5, 1.0, 1.0/400.0)
{
	_ahrs = ahrs; 
	_rangefinder = rangefinder; 
	_ins = ins; 
	_optflow = optflow; 
	_baro = baro; 
	_pitchComp = 0; 
	_rollComp = 0; 
	_desired_forward = _desired_right = 0; 
	_flow_front_filt.set_cutoff_frequency(5.0); 
	_flow_right_filt.set_cutoff_frequency(5.0); 
}

void RangeAvoid::set_vel_kP(float kp){
	_pitch_pid.kP(kp); 
	_roll_pid.kP(kp); 
}

void RangeAvoid::set_vel_kI(float ki){
	_pitch_pid.kI(ki); 
	_roll_pid.kI(ki); 
}

void RangeAvoid::set_vel_kD(float kd){
	_pitch_pid.kD(kd); 
	_roll_pid.kD(kd); 
}

void RangeAvoid::set_center_kP(float kp){
	_pitch_center_pid.kP(kp); 
	_roll_center_pid.kP(kp); 
}

void RangeAvoid::set_center_kI(float ki){
	_pitch_center_pid.kI(ki); 
	_roll_center_pid.kI(ki); 
}

void RangeAvoid::set_center_kD(float kd){
	_pitch_center_pid.kD(kd); 
	_roll_center_pid.kD(kd); 
}
void RangeAvoid::input_desired_velocity_ms(float forward, float right){
	_desired_forward = forward; 
	_desired_right = right; 
}

const Vector2f RangeAvoid::get_filtered_flow() {
	return Vector2f(_flow_front_filtered, _flow_right_filtered); 
}

void RangeAvoid::reset(){
	_pitch_pid.reset(); 
	_roll_pid.reset(); 
	_pitch_center_pid.reset(); 
	_roll_center_pid.reset(); 
}

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void RangeAvoid::update_flow_velocity(const Vector2f &flow_rate, float altitude, float dt){
	float flow_forward = flow_rate.x * altitude; 
	float flow_right = flow_rate.y * altitude; 
	
	_flow_front_filt.apply(flow_forward, dt); 
	_flow_right_filt.apply(flow_right, dt); 

	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;
	
	// fuse the acceleration and flow into a velocity using a complementary filter
	float w = (0.075 / (0.075 + dt)); 
	_flow_front_filtered = w * (_flow_front_filtered + accel.x * dt) + (1.0 - w) * flow_forward;  
	_flow_right_filtered = w * (_flow_right_filtered + accel.y * dt) + (1.0 - w) * flow_right;  

	// integrate flow velocity into a distance traveled (right now just a test)
	_flow_distance_front += _flow_front_filtered * dt; 
	_flow_distance_right += _flow_right_filtered * dt; 
}

void RangeAvoid::update_ekf(float dt){
	// calculate acceleration in xy plane 
	Vector3f accel = _ins->get_accel(); 
	float accel_len = accel.length(); 
	accel.x = (double)accel.x - (double)sin(_ahrs->pitch) * (double)accel_len; 
	accel.y = (double)accel.y + (double)sin(_ahrs->roll) * (double)accel_len;

	Vector3f center = _rangefinder->get_center_point_offset() * 0.01; 
	//xf.update(center.x, (center.x - prev_center.x) / dt, accel.x, dt); 
	float vel = _rangefinder->get_velocity_forward() * dt; 
	xf.update(center.x, vel, -accel.x, dt); 
	static KalmanFilter<1> sf; 
	math::Vector<1> input; 
	input(0) = xf.get_velocity(); 
	sf.update(input, math::Vector<1>(), dt); 
	_velocity = Vector3f(xf.get_velocity() - sf.get_prediction()(0), 0, 0); 
	//static float prev_center = 0;  
	//xf.update(center.x, (xf.get_center_offset() - prev_center) / dt, -accel.x, dt); 
	//prev_center = xf.get_center_offset(); 
}

Vector3f RangeAvoid::get_center_offset(){
	return Vector3f(xf.get_center_offset(), 0, 0); 
}

Vector3f RangeAvoid::get_velocity(){
	return _velocity; 
}

Vector2f RangeAvoid::get_wall_avoidance_velocity_compensation(){
	float clear_front = _rangefinder->get_front_clearance_cm(); 
	float clear_back = _rangefinder->get_back_clearance_cm(); 
	float clear_right = _rangefinder->get_right_clearance_cm(); 
	float clear_left = _rangefinder->get_left_clearance_cm(); 

	// try to avoid object by going in opposite direction
	Vector2f directions[4] = {Vector2f(clear_front, 0), Vector2f(-clear_back, 0), Vector2f(0, -clear_left), Vector2f(0, clear_right)};
	Vector2f closest_point(1000, 1000); 
	bool found = false; 
	for(int c = 0; c < 4; c++){
		float len = directions[c].length(); 
		if((len < closest_point.length()) && (len < RANGE_MIN_CLEARANCE)) {
			closest_point = directions[c]; 
			found = true; 
		}
	}

	// get the response vector which is same as closest point but with inverted length and pointing in the other direction
	if(found){
		float len = closest_point.length() / RANGE_MIN_CLEARANCE; // maximum length is min_clearance 
		closest_point.normalize(); // make it a unit vector pointing in the direction of the point.  
		closest_point *= (1.0 - len) * RANGE_AVOID_RESPONSE_SCALE; // scale it to the inverted length of the original vector.  
		return Vector2f(-constrain_float(closest_point.x, -0.5, 0.5), -constrain_float(closest_point.y, -0.5, 0.5));  
	}

	return Vector2f(0, 0); 
}

void RangeAvoid::update(float dt){
	if(is_zero(dt)) return; 

	// reset distance integral if pilot takes control 
	if(!is_zero(_desired_forward)) _flow_distance_front = 0; 
	if(!is_zero(_desired_right)) _flow_distance_right = 0; 

	update_ekf(dt); 

	// only use flow if we have flow and at least baro or rangefinder altitude
	if( 0 ) { //_optflow->healthy() && (_rangefinder->have_bottom() || _baro->healthy())) {
		// get the flow from the flow sensor
		Vector2f flow_rate = _optflow->flowRate(); 
		float altitude = 0; 

		// update our barometer zero point while we have rangefinder, after that use only barometer altitude relative to the previously sensed ground. 
		// Note: this may not work if suddenly ground gets closer, but is still out of rangefinder range. Then flow velocity will be slightly off. 
		if(_rangefinder->have_bottom()) {
			altitude = _rangefinder->get_bottom_clearance_cm() * 0.01; 
			_baro_zero_altitude = _baro->get_altitude() - altitude; 
		} else {
			altitude = _baro->get_altitude() - _baro_zero_altitude; 
		}

		update_flow_velocity(flow_rate, altitude, dt); 

		// add velocity based wall avoidance to the pilot inputs
		//Vector2f wall_avoid = get_wall_avoidance_velocity_compensation(); 
		//_desired_forward += wall_avoid.x; 
		//_desired_right += wall_avoid.y; 

		_pitch_pid.set_input_filter_all(_desired_forward - _flow_front_filtered); 
		_roll_pid.set_input_filter_all(_desired_right - _flow_right_filtered); 

		_output_pitch = -constrain_float(_pitch_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
		_output_roll = constrain_float(_roll_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

		_pitch_center_pid.reset(); 
		_roll_center_pid.reset(); 

		/*hal.console->printf("WALL_AVOID OUT[ %f %f ], AVOID[ %f %f ], ALT[ %f ]\n", 
			(double)_output_pitch, (double)_output_roll, 
			(double)wall_avoid.x, (double)wall_avoid.y, 
			(double)altitude);   
		*/
	} else { //if(_rangefinder->have_center_point()) {
		// if we are flying and we have a center point offset but no velocity measurements from optical flow then we try to center the copter between the four walls	
		Vector3f center = Vector3f(_velocity.x, 0, 0); //_rangefinder->get_center_point_offset() * 0.01; 
	
		center.x = constrain_float(center.x, -0.5, 0.5); 
		center.y = constrain_float(center.y, -0.5, 0.5); 

		_pitch_center_pid.set_input_filter_all(_desired_forward + center.x); 
		_roll_center_pid.set_input_filter_all(_desired_right + center.y); 

		_output_pitch = -constrain_float(_pitch_center_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);
		_output_roll = constrain_float(_roll_center_pid.get_pid(), -RANGE_MAX_RESPONSE, RANGE_MAX_RESPONSE);

		/*hal.console->printf("CENTER OUT[ %f %f ], OFF[ %f %f ]\n", 
			(double)_output_pitch, (double)_output_roll, 
			(double)center.x, (double)center.y);   
			*/
	} /* else {
		// otherwise just use pilot inputs scaled by P
		_output_pitch = -_desired_forward * _pitch_pid.kP(); 
		_output_roll = _desired_right * _roll_pid.kP(); 

		// reset pid controller integrals as well
		_pitch_pid.reset(); 
		_roll_pid.reset();

		_pitch_center_pid.reset(); 
		_roll_center_pid.reset(); 

		//hal.console->printf("NO_VEL OUT[ %f %f ]\n", 
	//		(double)_output_pitch, (double)_output_roll);   
	}*/

	//hal.console->printf("%f %f %f %f %f %f %f %f %f %f %f\n", clear_front, clear_back, _flow_distance_front, _flow_distance_right, clear_bottom, desired_forward, desired_right, 
	//		_flow_front_filtered, _flow_right_filtered, _rangefinder->get_velocity_forward() / 100.0, _rangefinder->get_velocity_right() / 100.0); 
	//, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 
	//hal.console->printf("%f %f %f %f %f %f %f %f %f\n", flow_rate.x, flow_rate.y, flow_forward, flow_right, _flow_front_filtered, _flow_right_filtered, accel.x, accel.y, clear_bottom); 
}

RangeAvoid::RangeFilter::RangeFilter(){	
	// user input weight matrix
	B = math::Matrix<3, 3>((const float[3][3]){
		{1.0, 0.0, 0.0}, 
		{0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0}
	}); 
	// input vector covariance matrix 
	H = math::Matrix<3, 3>((const float[3][3]){
		{1.0, 0.0, 0.0}, 
		{0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0} 
	}); 
	// gain matrix (computed at each frame)
	K = math::Matrix<3, 3>((const float[3][3]){
		{1.0, 0.0, 0.0}, 
		{0.0, 1.0, 0.0}, 
		{0.0, 0.0, 1.0}
	}); 
	// prediction error matrix
	P = math::Matrix<3, 3>((const float[3][3]){
		{1.0, 0.0, 0.0}, 
		{0.0, 1.0, 0.0},
		{0.0, 0.0, 1.0}
	}); 
	// sensor noise
	R = math::Matrix<3, 3>((const float[3][3]){
		{0.2, 0.0, 0.0}, 
		{0.0, 20.0, 0.0},
		{0.0, 0.0, 0.2}
	}); 
	// process noise 
	Q = math::Matrix<3, 3>((const float[3][3]){
		{0.0001, 0.0, 0.0}, 
		{0.0, 0.0001, 0.0},
		{0.0, 0.0, 0.01}
	}); 

	xk = math::Vector<3>((const float[3]){0.0, 0.0, 0.0}); 
}

void RangeAvoid::RangeFilter::update(float pos, float vel, float acc, float dt){
	math::Vector<3> zk(pos, vel, acc); 
	math::Vector<3> uk(0.0, 0.0, 0.0); 

	// update state transition matrix
	float F_mat[3][3] = {
		{1.0, dt,  0.0}, 
		{0.0, 1.0, dt },
		{0.0, 0.0, 1.0}
	}; 
	F = math::Matrix<3, 3>(F_mat); 

	// predict
	xk = F * xk + B * uk; 
	P = F * P * F.transposed() + Q; 

	// observe
	math::Vector<3> innovation = zk - H * xk; 
	math::Matrix<3, 3> innovation_cov = H * P * H.transposed() + R; 

	// update
	K = P * H.transposed() * innovation_cov.inversed(); 
	xk = xk + K * innovation; 
	P = P - K * H * P; 
}

float RangeAvoid::RangeFilter::get_center_offset() const {
	return xk(0); 
}

float RangeAvoid::RangeFilter::get_velocity() const {
	return xk(1); 
}

float RangeAvoid::get_desired_pitch_angle(){
	return _output_pitch;  
}

float RangeAvoid::get_desired_roll_angle(){
	return _output_roll;  
}
