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

#include <matrix/matrix/Matrix.hpp>
#include <matrix/matrix/SquareMatrix.hpp>
#include <matrix/matrix/Vector.hpp>

class AC_MatrixPID {
public:
	AC_MatrixPID(); 
	// input of various sensor data
	void input_measured_position(const matrix::Vector3f &pos); 
	void input_measured_velocity(const matrix::Vector3f &vel); 
	void input_measured_acceleration(const matrix::Vector3f &acc); 
	void input_measured_angles(const matrix::Vector3f &ang); 
	void input_measured_angular_velocity(const matrix::Vector3f &omega); 
	void input_measured_angular_acceleration(const matrix::Vector3f &accel); 

	// input of target values 
	void input_target_position(const matrix::Vector3f &pos); 
	void input_target_velocity(const matrix::Vector3f &vel); 
	void input_target_angles(const matrix::Vector3f &ang); 
	void input_target_angular_velocity(const matrix::Vector3f &omega); 

	// reading outputs
	matrix::Vector3f get_desired_velocity(void); 
	matrix::Vector3f get_desired_acceleration(void); 
	matrix::Vector3f get_desired_angular_velocity(void); 
	matrix::Vector3f get_desired_angular_acceleration(void); 

	// setting gains
	void set_position_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd); 
	void set_velocity_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd); 
	void set_angle_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd); 
	void set_angular_velocity_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd); 

	void update(float dt); 	
private:
	matrix::Matrix<float, 3, 6> Z, Znew; // measured quantities
	matrix::Matrix<float, 3, 6> T, Tnew; // target quantities 
	matrix::Matrix<float, 6, 1> B; // boolean matrix specifying inputs that were added since last update  
	matrix::Matrix<float, 3, 6> I; // integrals  
	matrix::Matrix<float, 3, 6> O; // output  
	matrix::Matrix<float, 3, 6> Kp, Ki, Kd; // pid gains 
}; 
