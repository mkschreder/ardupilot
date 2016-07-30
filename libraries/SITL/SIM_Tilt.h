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

class TiltSim : public Aircraft {
public:
    TiltSim(const char *home_str, const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new TiltSim(home_str, frame_str);
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
		double pos[3]; 
		double vel[3]; 
		double euler[3]; 
		double acc[3]; 
    };

	// packets sent from tilt sim to us
    struct client_packet {
        double timestamp;
		double gyro[3]; 
		double accel[3]; 
		double euler[3]; 
		double pos[3]; 
		double vel[3]; 
		double rcin[8]; 
    };

    void send_state(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);
	void drain_control_socket(); 
	
	long long _packet_timeout; 
    SocketAPM sock;
	uint8_t _mode; 
    Frame *_frame;
};

} // namespace SITL
