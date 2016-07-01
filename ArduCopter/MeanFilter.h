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

template<unsigned int N>
class MeanFilter {
public: 
	MeanFilter(){
		for(unsigned int c = 0; c < N; c++){
			_samples[c] = 0; 
			_cf[c] = 1; 
		}
	}
	float update(float val){
		float csum = 0; 
		float ssum = 0; 
		shift(val); 
		for(unsigned int c = 0; c < N; c++){
			csum += _cf[c]; 
			ssum += _cf[c] * _samples[c]; 
		}
		if(csum > 0) // safety check
			_result = ssum / csum; 
		return _result; 
	}
	float get(){
		return _result; 
	}
private: 
	void shift(float val){
		for(unsigned int i = 1; i < N; i++){
			_samples[i-1] = _samples[i]; 
		}
		_samples[N-1] = val; 
	}

	float _samples[N]; 
	float _cf[N]; 
	float _result; 
}; 

