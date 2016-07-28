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
    sock.bind("127.0.0.1", 9003);

    sock.reuseaddress();
    sock.set_blocking(false);
}

void TiltSim::send_state(const struct sitl_input &input){
    servo_packet pkt;
	memset(&pkt, 0, sizeof(pkt)); 

	for(int c = 0; c < 8; c++){
		pkt.servo[c] = input.servos[c]; 
	}

	float r, p, y;
	dcm.to_euler(&r, &p, &y);
	
	pkt.euler[0] = r; pkt.euler[1] = p; pkt.euler[2] = y; 
	pkt.pos[0] = position.x; pkt.pos[1] = position.y; pkt.pos[2] = position.z; 

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
    fdm_packet pkt;
	memset(&pkt, sizeof(pkt), 0); 

	if(sock.recv(&pkt, sizeof(pkt), 0) == sizeof(pkt)){
		::printf("ax: %f\t%f\nay: %f\t%f\naz: %f\t%f\n", pkt.accel[0], accel_body.x, pkt.accel[1], accel_body.y, pkt.accel[2], accel_body.z); 
		::printf("gx: %f\t%f\ngy: %f\t%f\ngz: %f\t%f\n", pkt.gyro[0], gyro.x, pkt.gyro[1], gyro.y, pkt.gyro[2], gyro.z); 
		::printf("pos: %f %f %f\n", position.x, position.y, position.z); 
		::printf("rand %d\n", rand()); 
		::printf("\033[^H"); 
		accel_body = Vector3f(0, 0, -9.82 + (rand() % 100) / 100.0f - 0.5); //Vector3f(pkt.accel[0], pkt.accel[1], pkt.accel[2]); 
		gyro = Vector3f(pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]); 
		//printf("acc(%f %f %f)\n", accel_body.x, accel_body.y, accel_body.z); 
		rcin_chan_count = 8; 
		for(unsigned c = 0; c < 8; c++) rcin[c] = pkt.rcin[c]; 
		position = Vector3f(pkt.pos[0], pkt.pos[1], pkt.pos[2]); 
    	velocity_ef = Vector3f(pkt.vel[0], pkt.vel[1], pkt.vel[2]);
	}	

	//position.z = -0.5; 
    //dcm.from_euler(pkt.euler[0], pkt.euler[1], pkt.euler[2]);

	adjust_frame_time(1000);
    sync_frame_time();
	drain_control_socket(); 
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
    recv_fdm(input);

    // get wind vector setup
    //update_wind(input);

    //Vector3f rot_accel;
   	//_frame->calculate_forces(*this, input, rot_accel, accel_body);

    //update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
