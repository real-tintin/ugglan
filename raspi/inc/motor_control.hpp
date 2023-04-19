#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <array>
#include <cstdint>
extern "C"
{
#include <math.h>
}

#include <drone_props.hpp>

struct BodyControl
{
    double f_z; // [N]
    double m_x; // [Nm]
    double m_y; // [Nm]
    double m_z; // [Nm]
};

typedef std::array<int16_t, droneprops::N_MOTORS> MotorControl;

MotorControl body_to_motor_controls(BodyControl &body_controls);

#endif /* MOTOR_CONTROL_HPP */
