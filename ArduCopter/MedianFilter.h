#pragma once
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

static inline int _median_compare_floats(const void *a, const void *b){
	float _a = *((float*)a); 
	float _b = *((float*)b); 
	return (_a > _b) - (_a < _b); 
}

template<unsigned int N>
class MedianFilterBase {
public: 
	MedianFilterBase(){
		_result = 0; 
		memset(_samples, 0, sizeof(_samples)); 
	}
	float update(float val){
		float arr[N]; 
		shift(val); 
		memcpy(arr, _samples, sizeof(arr)); 
		qsort(arr, N, sizeof(float), _median_compare_floats); 
		_result = arr[N/2]; 
		return _result; 
	}
	float get() {
		return _samples[N/2]; 
	}
protected: 
	void shift(float val){
		for(unsigned int i = 1; i < N; i++){
			_samples[i-1] = _samples[i]; 
		}
		_samples[N-1] = val; 
	}
	float _result; 
	float _samples[N]; 
}; 

template<unsigned int N>
class MedianFilter : public MedianFilterBase<N> { }; 
/*
template<>
class MedianFilter<3>: public MedianFilterBase<3> {
public: 
	float update(float val){
		float t; 
		shift(val); 

		float a = _samples[0]; 
		float b = _samples[1]; 
		float c = _samples[2]; 
		if(a > b) {
			t = b; b = a; a = t; 
		}
		if(b > c) {
			t = c; c = b; b = t; 
		}
		if(a > b) {
			t = b; b = a; a = t; 
		}
		_result = b; 
		return _result; 
	}
}; 
*/
