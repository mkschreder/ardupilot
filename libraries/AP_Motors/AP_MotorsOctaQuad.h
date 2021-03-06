// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsOctaQuad.h
/// @brief	Motor control class for OctaQuadcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"   // Parent Motors Matrix library

/// @class      AP_MotorsOcta
class AP_MotorsOctaQuad : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsOctaQuad(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    { };

    // setup_motors - configures the motors for a quad
    virtual void        setup_motors();
protected:
};
