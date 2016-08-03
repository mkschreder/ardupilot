/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/* Simulation interface for the tilt sim */

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"
#include "SIM_Multicopter.h"

namespace SITL {

class QuadSim : public Aircraft {
public:
    QuadSim(const char *home_str, const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new QuadSim(home_str, frame_str);
    }

private:
	enum {
		MODE_CLIENT_SIM = 0, 
		MODE_SERVER_SIM = 1
	}; 
	// tilt sim servo packets
    struct server_packet {
		uint8_t mode; 
		int16_t servo[8]; 
		float pos[3]; 
		float vel[3]; 
		float euler[3]; 
		float acc[3]; 
		float mag[3]; 
    };

	// packets sent from tilt sim to us
    struct client_packet {
		uint32_t id; 
        float timestamp;
		float gyro[3]; 
		float accel[3]; 
		float euler[3]; 
		float pos[3]; 
		float vel[3]; 
		float rcin[8]; 
		int32_t loc[3]; 
		float mag[3]; 
		float range[6]; 
    };

    void send_state(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);
	void drain_control_socket(); 
	
	long long _packet_timeout; 
    //SocketAPM sock;
	uint8_t _mode; 
    Frame *_frame;
	uint32_t _last_packet_id; 

	char *_shmout, *_shmin; 
};

} // namespace SITL
