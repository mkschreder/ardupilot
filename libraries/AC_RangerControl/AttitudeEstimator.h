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

#include <matrix/matrix/Vector.hpp>
#include <matrix/matrix/Matrix.hpp>
#include <matrix/matrix/Quaternion.hpp>

class AttitudeEstimator {
public: 
	AttitudeEstimator(); 
	void input_measured_gyro_rates(const matrix::Vector3f &gyro); 
	void input_measured_acceleration(const matrix::Vector3f &accel); 
	const matrix::Quaternion<float> &get_estimated_orientation(); 
	void update(float dt); 
private:
	// estimated bias
	matrix::Vector3f _gyro_bias; 
	// last gyro readings 
	matrix::Vector3f _w; 
	// last accelerometer readings
	matrix::Vector3f _a; 
	// estimated orientation 
	matrix::Quaternion<float> _q; 
	// error state jacobian
	matrix::Matrix<float, 3, 3> F; 
	// system covariance matrix
	matrix::Matrix<float, 3, 3> P; 
	// accelerometer measurement covariance
	matrix::Matrix<float, 3, 3> aCov; 
	// gyro measurement covariance
	matrix::Matrix<float, 3, 3> wCov; 
	// system stable 
	bool _is_stable; 
}; 
