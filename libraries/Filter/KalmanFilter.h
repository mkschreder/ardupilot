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

template<unsigned int M, unsigned int N>
class KalmanFilter {
public: 
	KalmanFilter(){
		F.setIdentity(); 
		B.setZero(); 
		H.setZero(); 
		P.setIdentity(); 
		R.setIdentity(); 
		Q.setIdentity(); 
		xk.setZero(); 
	}
	void update(const matrix::Vector<float, M> &zk, const matrix::Vector<float, N> &uk){
		// predict
		xk = F * xk + B * uk; 
		P = F * P * F.transposed() + Q; 

		// observe
		matrix::Vector<float, M> innovation = zk - H * xk; 
		matrix::Matrix<float, M, M> innovation_cov = H * P * H.transposed() + R; 

		// update
		matrix::Matrix<float, N, M> K = P * H.transposed() * matrix::inversed(innovation_cov); 
		xk = xk + K * innovation; 
		P = P - K * H * P; 
	}
	void set_state_transition_matrix(const matrix::Matrix<float, N, N> &mat){
		F = mat; 
	}
	void set_sensor_covariance_matrix(const matrix::Matrix<float, M, M> &mat){
		R = mat; 
	}
	void set_state_covariance_matrix(const matrix::Matrix<float, N, N> &mat){
		Q = mat; 
	}
	void set_state_input_matrix(const matrix::Matrix<float, M, N> &mat){
		H = mat; 
	}
	const matrix::Vector<float, N> &get_prediction() const {
		return xk; 
	}
private: 
	// state transition matrix
	matrix::Matrix<float, N, N> F; 
	// external forces matrix
	matrix::Matrix<float, N, N> B;
	// input vector to state matrix 
	matrix::Matrix<float, M, N> H;
	// sensor noise
	matrix::Matrix<float, M, M> R; 
	// process noise 
	matrix::Matrix<float, N, N> Q; 

	// prediction error matrix
	matrix::Matrix<float, N, N> P;  

	// filter state 
	matrix::Vector<float, N> xk; 
}; 

