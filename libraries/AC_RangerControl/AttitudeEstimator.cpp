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

#include "AttitudeEstimator.h"
#include <kalman/include/KalmanFilter.hpp>
#include <matrix/matrix/Quaternion.hpp>
using namespace Eigen; 
using namespace filter; 

#define FUSE_GYRO 	(1 << 0)
#define FUSE_ACC	(1 << 1)

class AttitudeEstimator::Data : public IUKFModel<10, 6> {
	/************************
		Filter state
		-------------
		0-2: 	w
		3-5: 	wb
		6-9: 	q	
		-------------
	*************************/	
	UnscentedKalmanFilter<10, 6, 1> _k; 
	Matrix<float, 3, 1> _gyro_in; 
	Matrix<float, 3, 1> _acc_in; 
	unsigned int _flags; 
public: 
	Data();
	virtual ~Data(){}; 
	void input_measured_gyro_rates(float wx, float wy, float wz); 
	void input_measured_acceleration(float ax, float ay, float az); 
	void get_estimated_quaternion(float (&q)[4]); 
	void get_estimated_omega(float (&w)[3]); 
	void update(float dt); 
		
	virtual Matrix<float, 10, 1> F(const Matrix<float, 10, 1> &i) override; 
	virtual Matrix<float, 6, 1> H(const Matrix<float, 10, 1> &i) override; 

private:
	void _updateQ(); 

	Matrix<float, 3, 1> _state_angular_rate(const Matrix<float, 10, 1> &s){
		Matrix<float, 3, 1> M; 
		M << s(0), s(1), s(2); 
		return M; 
	}

	Matrix<float, 3, 1> _state_gyro_bias(const Matrix<float, 10, 1> &s){
		Matrix<float, 3, 1> M; 
		M << s(3), s(4), s(5); 
		return M; 
	}

	Quaternion<float> _state_quaternion(const Matrix<float, 10, 1> &s){
		return Quaternion<float>(s(6), s(7), s(8), s(9)); 
	}
}; 

AttitudeEstimator::Data::Data() : _k(this){
	_flags = 0; 
	_gyro_in.setZero(); 
	_acc_in.setZero(); 

	// set initial state
	_k.set_xk(Matrix<float, 10, 1>((const float[]){
		0.0, 0.0, 0.0, 		//w
		0.0, 0.0, 0.0, 		//wb
		1.0, 0.0, 0.0, 0.0	//q
	})); 

	// setup initial belief
	Matrix<float, 10, 10> P; 
	Matrix<float, 10, 10> Q; 
	Matrix<float, 6, 6> R; 
	P << 
		0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1; 
	R << 
		0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0; 

	// setup initial covariance 
	_k.set_P(P); 

	// setup measurement covariance 
	_k.set_R(R); 
}

void AttitudeEstimator::Data::input_measured_gyro_rates(float wx, float wy, float wz){
	_gyro_in(0) = wx; 
	_gyro_in(1) = wy; 
	_gyro_in(2) = wz; 
	_flags |= FUSE_GYRO; 
}

void AttitudeEstimator::Data::input_measured_acceleration(float ax, float ay, float az){
	_acc_in(0) = ax; 
	_acc_in(1) = ay; 
	_acc_in(2) = az; 
	_flags |= FUSE_ACC; 
}

void AttitudeEstimator::Data::get_estimated_quaternion(float (&q)[4]){
	Matrix<float, 10, 1> x = _k.get_prediction(); 
	Quaternion<float> _q = _state_quaternion(x);  
	printf("STATE Q: %f %f %f %f\n", _q.w(), _q.x(), _q.y(), _q.z()); 
	_q.normalize(); 
	q[0] = _q.w(); 
	q[1] = _q.x(); 
	q[2] = _q.y(); 
	q[3] = _q.z(); 
}

void AttitudeEstimator::Data::get_estimated_omega(float (&w)[3]){
	Matrix<float, 10, 1> x = _k.get_prediction(); 
	Matrix<float, 3, 1> o = _state_angular_rate(x); 
	w[0] = o(0); 
	w[1] = o(1); 
	w[2] = o(2); 
}

void AttitudeEstimator::Data::_updateQ(){
	Matrix<float, 10, 10> Q; 
	Matrix<float, 3, 3> qr; 
	Matrix<float, 10, 1> x = _k.get_prediction(); 
	Quaternion<float> q = _state_quaternion(x); 
	qr << 
		q.w(), -q.z(), q.y(), 
		q.z(), q.w(), -q.x(), 
		-q.y(), q.x(), q.w(); 
	Matrix<float, 3, 1> w; 
	w << 
		0.1, 0.1, 0.1; 
	qr = 1.0f/4.0f * qr * w.asDiagonal() * qr.transpose(); 
	Q << 
		0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01;  
	
	_k.set_Q(Q); 
}

void AttitudeEstimator::Data::update(float dt){
	_updateQ(); 

	_k.predict(Matrix<float, 1, 1>()); 

	// TODO: separate gyro and acc fusing, now they are done in one step
	if(_flags & (FUSE_GYRO | FUSE_ACC)){
		printf("fuse gyro: %f %f %f, acc: %f %f %f\n", 
			_gyro_in(0), _gyro_in(1), _gyro_in(2), 
			_acc_in(0), _acc_in(1), _acc_in(2)); 
		Matrix<float, 6, 1> zk; 
		zk(0) = _gyro_in(0); 
		zk(1) = _gyro_in(1); 
		zk(2) = _gyro_in(2); 
		zk(3) = _acc_in(0); 
		zk(4) = _acc_in(1); 
		zk(5) = _acc_in(2); 

		_k.update(zk); 

		_flags = 0; 
	}
}

Matrix<float, 10, 1> AttitudeEstimator::Data::F(const Matrix<float, 10, 1> &i) {
	Matrix<float, 10, 1> data; 
	Matrix<float, 3, 1> b = _state_gyro_bias(i); 
	Matrix<float, 3, 1> w = _state_angular_rate(i) - b; 
	matrix::Quaternion<float> qq(i(6), i(7), i(8), i(9)); 
	qq.applyRates(matrix::Vector3f(w(0), w(1), w(2)), 1.0f/400.0f); 
	data <<
		i(0), i(1), i(2), 
		i(3), i(4), i(5), 
		qq(0), qq(1), qq(2), qq(3); 
	return data; 
}

Matrix<float, 6, 1> AttitudeEstimator::Data::H(const Matrix<float, 10, 1> &i) {
	Matrix <float, 6, 1> data; 
	Matrix<float, 3, 1> _w = _state_angular_rate(i);  
	Matrix<float, 3, 1> _wb = _state_gyro_bias(i);  
	Matrix<float, 3, 1> gyr = _w + _wb; 
	data << 
		gyr(0), gyr(1), gyr(2), 
		0, 0, 0; 	
	return data; 
}

//** Public interface **/

AttitudeEstimator::AttitudeEstimator(){
	_data = new Data(); 
}

AttitudeEstimator::~AttitudeEstimator(){
	delete _data; 
}

void AttitudeEstimator::input_measured_gyro_rates(float wx, float wy, float wz){
	_data->input_measured_gyro_rates(wx, wy, wz); 
}

void AttitudeEstimator::input_measured_acceleration(float ax, float ay, float az){
	_data->input_measured_acceleration(ax, ay, az); 
}

void AttitudeEstimator::get_estimated_quaternion(float (&q)[4]){
	_data->get_estimated_quaternion(q); 
}

void AttitudeEstimator::get_estimated_omega(float (&w)[3]){
	_data->get_estimated_omega(w); 
}

void AttitudeEstimator::update(float dt){
	_data->update(dt); 
}

