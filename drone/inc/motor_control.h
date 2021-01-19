#include <cstdint>
#include <math.h>
#include <array>

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

inline const uint8_t N_MOTORS = 4;

struct BodyControl
{
    double f_z; // [N]
    double m_x; // [Nm]
    double m_y; // [Nm]
    double m_z; // [Nm]
};

typedef std::array<int16_t, N_MOTORS> MotorControl;

MotorControl body_to_motor_controls(BodyControl& body_controls);

#endif /* MOTOR_CONTROL_H */
