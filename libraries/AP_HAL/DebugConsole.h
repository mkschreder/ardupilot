/* PX4 debug console class */
#pragma once

#include "utility/BetterStream.h"
#include "utility/print_vprintf.h"

class AP_HAL::DebugConsole : public AP_HAL::BetterStream {
public: 
	    /* Implementations of BetterStream virtual methods. These are
     * provided by AP_HAL to ensure consistency between ports to
     * different boards
     */
    void printf(const char *s, ...) FMT_PRINTF(2, 3);
    void vprintf(const char *s, va_list ap);

	// Stream methods
	virtual int16_t available() { return 0; };
    virtual int16_t txspace() { return 4096; };
    virtual int16_t read() { return -1; };

	// Print methods
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buffer, size_t size);
}; 

