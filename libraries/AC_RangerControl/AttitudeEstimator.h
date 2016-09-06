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

/******************
State: 
	o w = current angular velocity 
	x q = estimated orientation
	x a = estimated acceleration
	x v = estimated velocity 
	x p = estimated position
Measurement: 
	w = angular velocity in rad/s
	a = acceleration in m/s/s (aL + g)
H function: 
	w = wState
	a = inv(q) * aState
F function: 
	w = w 
	q = q + 1/2 q x w
	a = dq * 
********************/

class AttitudeEstimator {
public: 
	AttitudeEstimator(); 
	~AttitudeEstimator(); 

	void input_measured_gyro_rates(float wx, float wy, float wz); 
	void input_measured_acceleration(float ax, float ay, float az); 
	void get_estimated_quaternion(float (&q)[4]); 
	void get_estimated_omega(float (&w)[3]); 
	void update(float dt); 
private: 
	class Data; 
	Data *_data; 
}; 
