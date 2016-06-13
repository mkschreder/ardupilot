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

	//Vector2f velocity = optflow.bodyRate(); 
	//Vector2f flow_rate = optflow.flowRate(); 
	// INKO_TILT: here we have to feed pilot rc control 0 because we will then apply rc control to the servo directly. 
	// INKO_TILT: in tilt mode, motor speeds are not effected by pilot pitch input. 

	float rc_p = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.2; 
	float rc_i = 0; //constrain_float((hal.rcin->read(5) - 1000.0) * 1.0, 0, 1000) * 0.001; 
	float rc_d = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.02; 

	range_avoid.set_kP(rc_p); 
	range_avoid.set_kI(rc_i); 
	range_avoid.set_kD(rc_d); 

	range_avoid.input_desired_velocity_ms(target_pitch / 4500.0 * 0.5, target_roll / 4500.0 * 0.5); 

	rangefinders.update(G_Dt); 
	range_avoid.update(G_Dt); 

	target_pitch = range_avoid.get_desired_pitch_angle() * 100.0; 
	target_roll = range_avoid.get_desired_roll_angle() * 100.0; 

	Vector2f flow = optflow.flowRate(); 
	//hal.console->printf("ALT: %d, VRIGHT: %f, pc: %f, rc: %f\n", (int)range_bottom, rangefinders.get_velocity_right(), caPitchComp, caRollComp); 
	hal.console->printf("%d %f %f %f %f %f %f\n", AP_HAL::millis(), 
		range_avoid.get_desired_pitch_angle(), range_avoid.get_desired_roll_angle(), range_avoid.get_filtered_flow().x, range_avoid.get_filtered_flow().y, -flow.x, flow.y); 
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

	target_pitch = constrain_float(target_pitch, -4500, 4500); 
	target_roll = constrain_float(target_roll, -4500, 4500);

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
