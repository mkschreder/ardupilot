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

/* 
	Matrix PID is a vehicle pid controller that implements pid controls for
	position and rotation of the vehicle. 

	It deals only with values in body frame (it does not do any conversions!)

	It maintains a number of matrices that are all arranged according to
	following layout: 

	[ px, py, pz ] - position
	[ vx, vy, vz ] - velocity
	[ ax, ay, az ] - acceleration
	[ ox, oy, oz ] - angles (roll, pitch, yaw) radians
	[ rx, ry, rz ] - rates (roll, pitch, yaw) radians 
	[ xx, xy, xz ] - angular accelerations

	You use this controller by repeatedly inputting targets to it along with
	measurements. If you input any of the derivatives of quantities (for
	example velocity, which is derivative of position), then corresponding
	calculations will use your measured derivative instead of calculating a
	derivative based on change in quantities. This means that if you for
	example input angular rates (gyro) then these rates will be used to
	calculate error derivative instead of using changes in the actual angle.  
	
	Under the hood it is a pretty simple controller, but it saves
	a lot of coding space by grouping everything together in one place. 
*/
#include "AC_MatrixPID.h"

AC_MatrixPID::AC_MatrixPID(){
	Z.setZero(); Znew.setZero(); 
	T.setZero(); Tnew.setZero(); 
	I.setZero(); O.setZero(); 
	B.setZero(); 
	Kp.setZero(); Ki.setZero(); Kd.setZero(); 
}

// input of various sensor data
void AC_MatrixPID::input_measured_position(const matrix::Vector3f &pos){
	Znew.setCol(0, pos); 
	B(0, 0) = 1; 
}

void AC_MatrixPID::input_measured_velocity(const matrix::Vector3f &vel){
	Znew.setCol(1, vel); 
	B(0, 1) = 1; 
}

void AC_MatrixPID::input_measured_acceleration(const matrix::Vector3f &acc){
	Znew.setCol(2, acc); 
	B(0, 2) = 1; 
}

void AC_MatrixPID::input_measured_angles(const matrix::Vector3f &ang){
	Znew.setCol(3, ang); 
	B(0, 3) = 1; 
}
	
void AC_MatrixPID::input_measured_angular_velocity(const matrix::Vector3f &omega){
	Znew.setCol(4, omega); 
	B(0, 4) = 1; 
}
	
void AC_MatrixPID::input_measured_angular_acceleration(const matrix::Vector3f &acc){
	Znew.setCol(5, acc); 
	B(0, 5) = 1; 
}

// input of target values 
void AC_MatrixPID::input_target_position(const matrix::Vector3f &pos){
	Tnew.setCol(0, pos); 
}

void AC_MatrixPID::input_target_velocity(const matrix::Vector3f &vel){
	Tnew.setCol(1, vel); 
}

void AC_MatrixPID::input_target_angles(const matrix::Vector3f &ang){
	Tnew.setCol(3, ang); 
}

void AC_MatrixPID::input_target_angular_velocity(const matrix::Vector3f &omega){
	Tnew.setCol(4, omega); 
}

// reading outputs
matrix::Vector3f AC_MatrixPID::get_desired_velocity(void){
	// return output of position pid
	return O.getCol(0); 
}

matrix::Vector3f AC_MatrixPID::get_desired_acceleration(void){
	return O.getCol(1); 
}

matrix::Vector3f AC_MatrixPID::get_desired_angular_velocity(void){
	return O.getCol(3); 
}

matrix::Vector3f AC_MatrixPID::get_desired_angular_acceleration(void){
	return O.getCol(4);
}

void AC_MatrixPID::set_position_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd){
	Kp.setCol(0, kp); 
	Ki.setCol(0, ki); 
	Kd.setCol(0, kd);
}

void AC_MatrixPID::set_velocity_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd){
	Kp.setCol(1, kp); 
	Ki.setCol(1, ki); 
	Kd.setCol(1, kd);
}
 
void AC_MatrixPID::set_angle_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd){
	Kp.setCol(3, kp); 
	Ki.setCol(3, ki); 
	Kd.setCol(3, kd);
}

void AC_MatrixPID::set_angular_velocity_tuning(const matrix::Vector3f &kp, const matrix::Vector3f &ki, const matrix::Vector3f &kd){
	Kp.setCol(4, kp); 
	Ki.setCol(4, ki); 
	Kd.setCol(4, kd);
}

void AC_MatrixPID::update(float dt){
	// calculate error
	matrix::Matrix<float, 3, 6> E = T - Z; 

	// update integral
	I = I + E * dt; 

	// calculate error derivative based on measurements and inputs
	matrix::Matrix<float, 3, 6> D = (Tnew - T) - (Znew - Z); 

	// update derivatives based on actual measured data (if we have derivative measurements)
	if(B(0, 1)) D.setCol(0, (Tnew.getCol(0) - T.getCol(0)) - Znew.getCol(1)); // position error derivative 
	if(B(0, 2)) D.setCol(1, (Tnew.getCol(1) - T.getCol(1)) - Znew.getCol(2)); // velocity error derivative 
	if(B(0, 4)) D.setCol(3, (Tnew.getCol(3) - T.getCol(3)) - Znew.getCol(4)); // angle error derivative 
	if(B(0, 5)) D.setCol(4, (Tnew.getCol(4) - T.getCol(4)) - Znew.getCol(5)); // angle rate error derivative 

	// calculate output
	O = Kp.emult(E) + Ki.emult(I) + Kd.emult(D); 

	// update stored targets and measurements
	T = Tnew; 
	Z = Znew; 

	// reset the B matrix
	B.setZero(); 
}

