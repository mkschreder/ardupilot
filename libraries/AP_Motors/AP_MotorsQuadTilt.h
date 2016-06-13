// Tilt rotor motor class. Author: Martin Schröder <mkschreder.uk@gmail.com>

#pragma once

#include "AP_MotorsQuad.h"

/// @class      AP_MotorsQuadTilt
class AP_MotorsQuadTilt : public AP_MotorsQuad {
public:
    /// Constructor
    AP_MotorsQuadTilt(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsQuad(loop_rate, speed_hz)
    {
		AP_Param::setup_object_defaults(this, var_info); 
	};

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();
	
	// outputs values to servos
	virtual void 		output_to_servos(); 

	// params
	static const struct AP_Param::GroupInfo	var_info[]; 

	void 	set_motor_tilt(float tilt_deg) { _motor_tilt = tilt_deg; }
protected:
	
private: 
	AP_Int8 _servo_on; 
	AP_Int8 _servo_channel; 
	AP_Int16 _servo_travel; 

	float _motor_tilt; 
};


