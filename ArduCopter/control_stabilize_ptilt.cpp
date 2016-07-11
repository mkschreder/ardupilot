#include "Copter.h"

// same controller as the stabi
bool Copter::stabilize_ptilt_init(bool ignore_checks){
	hal.console->printf("Stabilize init.\n"); 
	stabilize_init(ignore_checks); 
	range_avoid.reset(); 	

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors.rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

	return true; 
}

FILE *logfile = 0; 

// stabilize_ptilt_run - runs the main stabilize controller
// pitch is not sent to the pid, rather it is later calcualted by the mixer from rc input and sent directly to the servo
// roll and yaw compensation however needs to be done here. 
// otherwise it does pretty much the same job as the stabilization controller
// should be called at 100hz or more
void Copter::stabilize_ptilt_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;

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

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) && motors.spool_up_complete());
#endif

    AltHoldModeState althold_state;

    // Alt Hold State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (!ap.auto_armed){
        althold_state = AltHold_NotAutoArmed;
    } else if (takeoff_state.running || takeoff_triggered){
        althold_state = AltHold_Takeoff;
    } else if (ap.land_complete){
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // get pilot's desired throttle
    //pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

	//Vector2f velocity = optflow.bodyRate(); 
	//Vector2f flow_rate = optflow.flowRate(); 
	// INKO_TILT: here we have to feed pilot rc control 0 because we will then apply rc control to the servo directly. 
	// INKO_TILT: in tilt mode, motor speeds are not effected by pilot pitch input. 
	float rc_4 = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.001; 
	float rc_5 = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.001; 

	float ku = 20 + rc_4 * 40;  
	float tu = 5 + rc_5 * 10; 

	float rc_vel_p = constrain_float((hal.rcin->read(4) - 1000.0), 0, 1000) * 0.08; // 6.32
	float rc_vel_i = 0;//constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; // 5.92 
	float rc_vel_d = constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; // 3.73 
	
	float rc_center_p = 0.6 * ku; 
	float rc_center_i = (1.2 * ku) / tu; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.01; 
	float rc_center_d = (0.6 * ku * tu) / 8; //constrain_float((hal.rcin->read(5) - 1000.0), 0, 1000) * 0.008; 

	//hal.console->printf("%f, %f, %f\n", rc_center_p, rc_center_i, rc_center_d); 

	range_avoid.set_vel_kP(rc_vel_p); 
	range_avoid.set_vel_kI(rc_vel_i); 
	range_avoid.set_vel_kD(rc_vel_d); 
	
	range_avoid.set_center_kP(rc_center_p); 
	range_avoid.set_center_kI(rc_center_i); 
	range_avoid.set_center_kD(rc_center_d); 

	rangefinders.update(G_Dt); 

	if(althold_state == AltHold_Flying){
		if(!logfile) {
			logfile = fopen("/fs/microsd/log.hex", "a"); 
			if(logfile) printf("Opened logfile\n"); 
		}
		range_avoid.input_desired_velocity_ms(-target_pitch / 4500.0 * 4, target_roll / 4500.0 * 4); 

		range_avoid.update(G_Dt); 

		target_pitch = range_avoid.get_desired_pitch_angle() * 100.0f; 
		target_roll = range_avoid.get_desired_roll_angle() * 100.0f; 
	} else {
		if(logfile){
			fclose(logfile); 
			printf("Closed logfile\n"); 
			logfile = 0; 
		}
		range_avoid.reset(); 
	}
/*	
	float center_offset = range_avoid.get_center_offset().x; 
	float front, back, right, left, bottom, top; 
	rangefinders.get_raw_readings_cm(&front, &back, &right, &left, &bottom, &top); 

	dbgConsole->printf("%f, %f, %f, %f\n", rangefinders.get_center_point_offset().x * 0.01, center_offset, range_avoid.get_velocity().x, rangefinders.get_velocity_forward() * 0.01); 
	*/
	//dbgConsole->printf("%f, %f, %f\n", front, back, rangefinders.get_front_clearance_cm()); 
	/*
	dbgConsole->printf("%d, ", AP_HAL::millis()); 
	float front, back, left, right, bottom, top; 
	rangefinders.get_raw_readings_cm(&front, &back, &left, &right, &bottom, &top); 
	// 2
	dbgConsole->printf("%f, %f, ", front, back); 
	Vector3f accel, gyro; 
	accel = ins.get_accel(); gyro = ins.get_gyro(); 
	// 4
	dbgConsole->printf("%f, %f, %f, ", accel.x, accel.y, accel.z); 
	// 7
	dbgConsole->printf("%f, ", ahrs.pitch_sensor / 100.0); 
	dbgConsole->printf("\n"); 
*/
	/*
	dbgConsole->printf("%d, ", AP_HAL::millis()); 
	float front, back, left, right, bottom, top; 
	rangefinders.get_raw_readings_cm(&front, &back, &left, &right, &bottom, &top); 
	// 2
	dbgConsole->printf("%f, %f, %f, %f, %f, ", front, back, left, right, bottom); 
	// 7
	dbgConsole->printf("%f, %f, %f, %f, %f, ", rangefinders.get_front_clearance_cm(), rangefinders.get_back_clearance_cm(),
		rangefinders.get_left_clearance_cm(), rangefinders.get_right_clearance_cm(), rangefinders.get_bottom_clearance_cm()); 
	Vector3f accel, gyro; 
	accel = ins.get_accel(); gyro = ins.get_gyro(); 
	// 12
	dbgConsole->printf("%f, %f, %f, %f, %f, %f, ", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z); 
	// 18
	dbgConsole->printf("%f, %f, %f, ", ahrs.yaw_sensor / 100.0, ahrs.pitch_sensor / 100.0, ahrs.roll_sensor / 100.0); 
	// 21
	dbgConsole->printf("%f, %f ", _optflow_rate.x, _optflow_rate.y); 
	dbgConsole->printf("\n"); 
	*/
	/*
		"%f %f %f %f %f ", 
		"%f %f %f %f %f\n", AP_HAL::millis(), 
			
			(double)rc_p, (double)rc_i, (double)rc_d, 
			(double)range_avoid.get_desired_pitch_angle(), (double)range_avoid.get_desired_roll_angle(), 
			(double)range_avoid.get_filtered_flow().x, (double)range_avoid.get_filtered_flow().y, 
			(double)-_optflow_rate.x, (double)_optflow_rate.y, 
			(double)(rangefinders.get_bottom_clearance_cm() * 0.01f)); 
	*/
	//hal.console->printf("Y: %f, P: %f, R: %f, kP: %f, kI: %f, kD: %f, BOTTOM: %f, FRONT: %f, BACK %f, LEFT: %f, RIGHT: %f, PC: %f, RC: %f\n", 
	//				target_yaw_rate, target_pitch, target_roll, rc_p, rc_i, rc_d, range_bottom, range_front, range_back, range_left, range_right, caPitchComp, caRollComp); 
	//hal.console->printf("FR: %f, BK: %f, comp: %f, tp: %f, np: %f, rc: %f, nroll: %f\n", range_front, range_back, pitComp, target_pitch, npitch, target_roll, rllComp); 

	// compensate yaw and roll
	//cliSerial->printf("target_yaw: %f, target_pitch: %f, target_roll: %f, ", target_yaw_rate, target_pitch, target_roll); 
	
#if FRAME_CONFIG == QUAD_PTILT_FRAME
	float cosAngle = cos(radians(target_pitch * 0.01)); 

	float rollComp = target_roll * cosAngle;
	float rollCompInv = target_roll - rollComp;
	float yawComp = target_yaw_rate * cosAngle;
	float yawCompInv = target_yaw_rate - yawComp;
	target_roll = yawCompInv + rollComp; 
	target_yaw_rate = yawComp + rollCompInv; 

	target_pitch = constrain_float(target_pitch, -4500, 4500); 
	target_roll = constrain_float(target_roll, -4500, 4500);
	
	// support body pitch on channel 4 so we can adjust body tilting when we fly. 
	// this will also rotate the tilt motors in the opposite direction. 
	// TODO: make all of this stuff configurable
	float bodyTilt = (hal.rcin->read(4) - 1500) / 500.0 * 90.0; 
	bodyTilt = 0; // disable for now
	float motorTilt = target_pitch * 0.01; 	

	motors.set_motor_tilt(motorTilt - bodyTilt); 

	// zero the target_pitch because we will be handling forward movement using motor tilt
	target_pitch = 0; 
#endif

    //attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, bodyTilt * 100, target_yaw_rate, get_smoothing_gain());

	// compensate throttle for motor tilt (throttle is 0-1.0 here)
	// compensating for rc input pitch + current body pitch because body may pitch as well and we need to adjust thrust for that as well 
	// total angle is constrained between 0 and 45 degrees. Beyond that we don't do compensation
	// the cos here is always between 0.7 and 1.0 so we don't need to worry about div by zero

	// disable throttle compensation for now because we are using altitude hold below
	//float compPitch = constrain_float(abs(ahrs.pitch_sensor * 0.01 + target_pitch * 0.01), 0, 45.0); 
	//pilot_throttle_scaled = pilot_throttle_scaled / cos(radians(compPitch)); 
	
	// ===========================================================
	// Run altitude hold logic here

    // output pilot's throttle
    //attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
    float takeoff_climb_rate = 0.0f;
	
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // get pilot desired lean angles
    //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    //float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME    
        // helicopters are capable of flying even with the motor stopped, therefore we will attempt to keep flying
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // force descent rate and call position controller
        pos_control.set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        pos_control.update_z_controller();
#else
        // Multicopters do not stabilize roll/pitch/yaw when motor are stopped
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
#endif
        break;

    case AltHold_NotAutoArmed:

        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
#if FRAME_CONFIG == HELI_FRAME
        // Helicopters always stabilize roll/pitch/yaw
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else
        // Multicopters do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:

#if FRAME_CONFIG == HELI_FRAME
        attitude_control.set_yaw_target_to_current_heading();
#endif
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->get_control_in()),false,g.throttle_filt);
        // set motors to spin-when-armed if throttle at zero, otherwise full range
        if (ap.throttle_zero) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->get_control_in())-throttle_average);
        break;

    case AltHold_Flying:
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }

}
