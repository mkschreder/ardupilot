#include "Copter.h"

// same controller as the stabi
bool Copter::stabilize_ptilt_init(bool ignore_checks){
	return stabilize_init(ignore_checks); 
}

// stabilize_ptilt_run - runs the main stabilize controller
// pitch is not sent to the pid, rather it is later calcualted by the mixer from rc input and sent directly to the servo
// roll and yaw compensation however needs to be done here. 
// otherwise it does pretty much the same job as the stabilization controller
// should be called at 100hz or more
void Copter::stabilize_ptilt_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

	// INKO_TILT: here we have to feed pilot rc control 0 because we will then apply rc control to the servo directly. 
	// INKO_TILT: in tilt mode, motor speeds are not effected by pilot pitch input. 
	//cliSerial->printf("target_pitch: %f\n", target_pitch); 

	float PITCH_CORRECT_FACTOR = constrain_float(hal.rcin->read(5) - 1000, 0, 1000) * 2; 
	float ROLL_CORRECT_FACTOR = constrain_float(hal.rcin->read(5) - 1000, 0, 1000) * 2; 

	// calculate pitch correction based on rangefinder sensor input
	/*float range_front = constrain_float(_rangefinder.filters[1].get(), 0, 200.0);  
	float range_back = constrain_float(_rangefinder.filters[0].get(), 0, 200.0);  
	float range_left = constrain_float(_rangefinder.filters[2].get(), 0, 200.0);  
	float range_right = constrain_float(_rangefinder.filters[3].get(), 0, 200.0);  
	float pitComp = constrain_float((range_back - range_front) / 200.0, -1.0, 1.0);  
	float rllComp = constrain_float((range_right - range_left) / 200.0, -1.0, 1.0);  
	// calculate final compensated pitch
	float caPitchComp = constrain_float(target_pitch + (pitComp * PITCH_CORRECT_FACTOR), -4500, 4500); 
	float caRollComp = constrain_float(target_roll + (rllComp * ROLL_CORRECT_FACTOR), -4500, 4500); 
	*/

	rangefinders.update(); 

	float range_front = rangefinders.get_front_clearance_cm(); //constrain_float(_rangefinder.filters[1].get(), 0, 200.0); 
	float range_back = rangefinders.get_back_clearance_cm(); //constrain_float(_rangefinder.filters[0].get(), 0, 200.0);  
	float range_right = rangefinders.get_right_clearance_cm(); //constrain_float(_rangefinder.filters[3].get(), 0, 200.0);  
	float range_left = rangefinders.get_left_clearance_cm(); //constrain_float(_rangefinder.filters[2].get(), 0, 200.0);  
	float range_bottom = rangefinders.get_bottom_clearance_cm(); 

	static AC_PID range_front_pid(0, 0, 0, 4500, 1, 1.0/400.0); 
	static AC_PID range_right_pid(0, 0, 0, 4500, 1, 1.0/400.0); 

	float rc_p = constrain_float((hal.rcin->read(4) - 1000.0) * 2.5, 0, 2000); 
	float rc_i = constrain_float((hal.rcin->read(5) - 1000.0) * 2.5, 0, 2000); 
	float rc_d = 0; //constrain_float((hal.rcin->read(5) - 1000.0) * 0.1, 0, 2000); 
/*	range_front_pid.kP(rc_p); 
	range_front_pid.kI(rc_i); 
	range_front_pid.kD(rc_d); 
	range_right_pid.kP(rc_p); 
	range_right_pid.kI(rc_i); 
	range_right_pid.kD(rc_d); 
	if(abs(target_yaw_rate) > 2000 || abs(target_pitch) > 1000 || abs(target_roll) > 1000) {
		//range_front_target = range_front; 
		//range_right_target = range_right; 
		//range_front_pid.reset_I(); 
		//range_right_pid.reset_I(); 
	}
*/
	float caPitchComp = 0; 
	float caRollComp = 0; 
	
	#define MIN_OBJECT_DISTANCE 60.0f
	if(!(range_front < MIN_OBJECT_DISTANCE && range_back < MIN_OBJECT_DISTANCE)){	
		if(range_front < MIN_OBJECT_DISTANCE) {
			caPitchComp = rc_p; 
		}
		if(range_back < MIN_OBJECT_DISTANCE) {
			caPitchComp = -rc_p; 
		}
	} 

	if(!(range_left < MIN_OBJECT_DISTANCE && range_right < MIN_OBJECT_DISTANCE)){
		if(range_right < MIN_OBJECT_DISTANCE) {
			caRollComp = -rc_p; //range_right_pid.get_pid(); 
		}
		if(range_left < MIN_OBJECT_DISTANCE) {
			caRollComp = rc_p; 
		}
	} 
/*
	if(!(range_front < MIN_OBJECT_DISTANCE && range_back < MIN_OBJECT_DISTANCE)){	
		if(range_front < MIN_OBJECT_DISTANCE) {
			range_front_pid.set_input_filter_all(MIN_OBJECT_DISTANCE - range_front); 
			caPitchComp = range_front_pid.get_pid(); 
		}
		if(range_back < MIN_OBJECT_DISTANCE) {
			range_front_pid.set_input_filter_all(-(MIN_OBJECT_DISTANCE - range_back)); 
			caPitchComp = range_front_pid.get_pid(); 
		}
	} else {
		range_front_pid.reset_I(); 
		range_front_pid.reset_filter(); 
		range_front_pid.set_input_filter_all(0); 
	}

	if(!(range_left < MIN_OBJECT_DISTANCE && range_right < MIN_OBJECT_DISTANCE)){
		if(range_right < MIN_OBJECT_DISTANCE) {
			range_right_pid.set_input_filter_all(-(MIN_OBJECT_DISTANCE - range_right)); 
			caRollComp = range_right_pid.get_pid(); 
		}
		if(range_left < MIN_OBJECT_DISTANCE) {
			range_right_pid.set_input_filter_all(MIN_OBJECT_DISTANCE - range_left); 
			caRollComp = range_right_pid.get_pid(); 
		}
	} else {
		range_right_pid.reset_I(); 
		range_right_pid.reset_filter(); 
		range_right_pid.set_input_filter_all(0); 
	}
*/
		
	//hal.console->printf("ALT: %d, VRIGHT: %f, pc: %f, rc: %f\n", (int)range_bottom, rangefinders.get_velocity_right(), caPitchComp, caRollComp); 
	static int count = 0; 
	
	hal.console->printf("%d %f %f %f\n", count++, rangefinders.get_back_clearance_cm(), rangefinders.get_velocity_forward(), rangefinders.get_raw_back()); 
	//hal.console->printf("Y: %f, P: %f, R: %f, kP: %f, kI: %f, kD: %f, BOTTOM: %f, FRONT: %f, BACK %f, LEFT: %f, RIGHT: %f, PC: %f, RC: %f\n", 
	//				target_yaw_rate, target_pitch, target_roll, rc_p, rc_i, rc_d, range_bottom, range_front, range_back, range_left, range_right, caPitchComp, caRollComp); 
	//hal.console->printf("FR: %f, BK: %f, comp: %f, tp: %f, np: %f, rc: %f, nroll: %f\n", range_front, range_back, pitComp, target_pitch, npitch, target_roll, rllComp); 

	// compensate yaw and roll
	//cliSerial->printf("target_yaw: %f, target_pitch: %f, target_roll: %f, ", target_yaw_rate, target_pitch, target_roll); 
	float cosAngle = cos(radians(target_pitch * 0.01)); 

	float rollComp = target_roll * cosAngle;
	float rollCompInv = target_roll - rollComp;
	float yawComp = target_yaw_rate * cosAngle;
	float yawCompInv = target_yaw_rate - yawComp;
	target_roll = yawCompInv + rollComp; 
	target_yaw_rate = yawComp + rollCompInv; 

	target_pitch = constrain_float(target_pitch + caPitchComp, -4500, 4500); 
	target_roll = constrain_float(target_roll + caRollComp, -4500, 4500);

	//hal.console->printf("P: %f, R: %f, PC: %f, RC: %f\n", target_pitch, target_roll, PITCH_CORRECT_FACTOR, ROLL_CORRECT_FACTOR); 

	// support body pitch on channel 4 so we can adjust body tilting when we fly. 
	// this will also rotate the tilt motors in the opposite direction. 
	// TODO: make all of this stuff configurable
	float bodyTilt = (hal.rcin->read(4) - 1500) / 500.0 * 90.0; 
	// switch position in the middle = no body tilt 
	int16_t sw = hal.rcin->read(5); 
	//if(sw > 1400 && sw < 1600) bodyTilt = 0; 
	bodyTilt = 0; // disable for now
	float motorTilt = target_pitch * 0.01; 	

	motors.set_motor_tilt(motorTilt - bodyTilt); 
	
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, bodyTilt * 100, target_yaw_rate, get_smoothing_gain());

	// compensate throttle for motor tilt (throttle is 0-1.0 here)
	// compensating for rc input pitch + current body pitch because body may pitch as well and we need to adjust thrust for that as well 
	// total angle is constrained between 0 and 45 degrees. Beyond that we don't do compensation
	// the cos here is always between 0.7 and 1.0 so we don't need to worry about div by zero
	float compPitch = constrain_int16(abs(ahrs.pitch_sensor * 0.01 + target_pitch * 0.01), 0, 45); 
	pilot_throttle_scaled = pilot_throttle_scaled / cos(radians(compPitch)); 

	//hal.console->printf("vel: %f, x: %f, y: %f, alt: %f\n", inertial_nav.get_velocity_xy(), 
//		inertial_nav.get_position().x, inertial_nav.get_position().y, inertial_nav.get_altitude()); 

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
