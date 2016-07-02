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

#include <mathlib/mathlib.h>

template<unsigned int M, unsigned int N>
class KalmanFilter {
public: 
	KalmanFilter(){
		F.identity(); 
		B.zero(); 
		H.zero(); 
		P.identity(); 
		R.identity(); 
		Q.identity(); 
		xk.zero(); 
	}
	void update(const math::Vector<M> &zk, const math::Vector<N> &uk){
		// predict
		xk = F * xk + B * uk; 
		P = F * P * F.transposed() + Q; 

		// observe
		math::Vector<M> innovation = zk - H * xk; 
		math::Matrix<M, M> innovation_cov = H * P * H.transposed() + R; 

		// update
		math::Matrix<N, M> K = P * H.transposed() * innovation_cov.inversed(); 
		xk = xk + K * innovation; 
		P = P - K * H * P; 
	}
	void set_state_transition_matrix(const math::Matrix<N, N> &mat){
		F = mat; 
	}
	void set_sensor_covariance_matrix(const math::Matrix<M, M> &mat){
		R = mat; 
	}
	void set_state_covariance_matrix(const math::Matrix<N, N> &mat){
		Q = mat; 
	}
	void set_state_input_matrix(const math::Matrix<M, N> &mat){
		H = mat; 
	}
	const math::Vector<N> &get_prediction() const {
		return xk; 
	}
private: 
	// state transition matrix
	math::Matrix<N, N> F; 
	// external forces matrix
	math::Matrix<N, N> B;
	// input vector to state matrix 
	math::Matrix<M, N> H;
	// sensor noise
	math::Matrix<M, M> R; 
	// process noise 
	math::Matrix<N, N> Q; 

	// prediction error matrix
	math::Matrix<N, N> P;  

	// filter state 
	math::Vector<N> xk; 
}; 

