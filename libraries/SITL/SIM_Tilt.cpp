/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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
  simulator connector for ardupilot version of TiltSim
*/

#include "SIM_Tilt.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

TiltSim::TiltSim(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    sock(true)
{
	_frame = NULL; 
	_mode = MODE_CLIENT_SIM; 
	_packet_timeout = 0; 
	if(frame_str){
		printf("Looking up frame %s\n", frame_str); 
		_frame = Frame::find_frame("+");
		if (_frame == NULL) {
			printf("Frame '%s' not found", frame_str);
			exit(1);
		}
		_frame->init(1.5, 0.5, 60, 4*radians(360));
		frame_height = 0.1;
		ground_behavior = GROUND_BEHAVIOR_NONE;
	}
    // try to bind to a specific port so that if we restart ArduPilot
    // TiltSim keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("127.0.0.1", 9005);

    sock.reuseaddress();
    sock.set_blocking(false);

	update_position();
	update_mag_field_bf();
}

void TiltSim::send_state(const struct sitl_input &input){
    server_packet pkt;
	memset(&pkt, 0, sizeof(pkt)); 

	for(int c = 0; c < 8; c++){
		pkt.servo[c] = input.servos[c]; 
	}

	float r, p, y;
	dcm.to_euler(&r, &p, &y);

	Vector3f velocity = dcm.transposed() * velocity_ef; 
	pkt.mode = _mode; 
	pkt.euler[0] = r; pkt.euler[1] = p; pkt.euler[2] = y; 
	pkt.pos[0] = position.x; pkt.pos[1] = position.y; pkt.pos[2] = position.z; 
	pkt.vel[0] = velocity.x; pkt.vel[1] = velocity.y; pkt.vel[2] = velocity.z; 
	pkt.acc[0] = accel_body.x; pkt.acc[1] = accel_body.y; pkt.acc[2] = accel_body.z; 
	pkt.mag[0] = mag_bf.x; pkt.mag[1] = mag_bf.y; pkt.mag[2] = mag_bf.z; 

    sock.sendto(&pkt, sizeof(pkt), "127.0.0.1", 9002);
}

#include <errno.h>

void TiltSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock.recv(buf, buflen, 0);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "error recv on control socket: %s",
                        strerror(errno));
            }
        } else {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);
}

void TiltSim::recv_fdm(const struct sitl_input &input){

}

/*
  update the TiltSim simulation by one time step
 */
void TiltSim::update(const struct sitl_input &input){
	long long tnow = AP_HAL::millis(); 
	if(tnow > _packet_timeout){ 
    	send_state(input);
		_packet_timeout = tnow + 10; 
	}

	client_packet pkt;
	memset(&pkt, sizeof(pkt), 0); 

	static long long _calls_since = 0; 
	_calls_since++; 
	if(sock.recv(&pkt, sizeof(pkt), 1) == sizeof(pkt)){
		if(_mode == MODE_CLIENT_SIM){
			accel_body = Vector3f(pkt.accel[0], pkt.accel[1], pkt.accel[2]); 
			gyro = Vector3f(pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]); 
			location.lat = pkt.loc[0]; 
			location.lng = pkt.loc[1]; 
			location.alt = pkt.loc[2]; 
			mag_bf = Vector3f(pkt.mag[0], pkt.mag[1], pkt.mag[2]); 
			//printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
			position = Vector3f(pkt.pos[0], pkt.pos[1], pkt.pos[2]); 
			velocity_ef = Vector3f(pkt.vel[0], pkt.vel[1], pkt.vel[2]);
		}

		if(pkt.id != (_last_packet_id + 1)) ::printf("DROPPED %d packets!\n", pkt.id - _last_packet_id); 
		else ::printf("packet %d\n", pkt.id); 
		_last_packet_id = pkt.id; 
		::printf("calls since last packet: %d\n", _calls_since); 
		_calls_since = 0; 
		::printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
		::printf("ax: %f\t%f\nay: %f\t%f\naz: %f\t%f\n", pkt.accel[0], accel_body.x, pkt.accel[1], accel_body.y, pkt.accel[2], accel_body.z); 
		::printf("gx: %f\t%f\ngy: %f\t%f\ngz: %f\t%f\n", pkt.gyro[0], gyro.x, pkt.gyro[1], gyro.y, pkt.gyro[2], gyro.z); 
		::printf("pos: %f %f %f\n", position.x, position.y, position.z); 
		::printf("vel: %f %f %f\n", velocity_ef.x, velocity_ef.y, velocity_ef.z); 
		::printf("loc: %d %d %d\n", location.lng, location.lat, location.alt); 
		::printf("loc_or: %f %f %f\n", pkt.loc[0], pkt.loc[1], pkt.loc[2]); 
		::printf("eu: %f %f %f\n", pkt.euler[0], pkt.euler[1], pkt.euler[2]); 
		Vector3f a = dcm * accel_body; 	
		::printf("accelef: %f %f %f\n", a.x, a.y, a.z); 

		rcin_chan_count = 8; 
		for(unsigned c = 0; c < 8; c++) rcin[c] = pkt.rcin[c]; 
	}	

	adjust_frame_time(1000);
    sync_frame_time();
	drain_control_socket(); 

	if(_mode == MODE_SERVER_SIM){
		// get wind vector setup
		update_wind(input);

		Vector3f rot_accel;

    	_frame->calculate_forces(*this, input, rot_accel, accel_body);

		update_dynamics(rot_accel);

		// update lat/lon/altitude
		update_position();

		// update magnetic field
		update_mag_field_bf();
	} else if(_mode == MODE_CLIENT_SIM){
		// update lat/lon/altitude
		update_position();

		// update magnetic field
		//update_mag_field_bf();
	}
}

} // namespace SITL
