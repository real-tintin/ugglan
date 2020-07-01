#include <cstdint>
#include <math.h>
#include <array>

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

static const uint8_t N_MOTORS = 4;

static const double INV_THRUST_CONST_FZ = 29869; // 1 / (4 * c_fz) [ rad/Ns]
static const double INV_THRUST_CONST_MZ = 129864; // 1 / (4 * l_x * c_fz) [rad/Ns]
static const double INV_TORQUE_CONST_MZ = 1493429; // 1 / (4 * c_mz) [rad/Nms]

static const double BODY_TO_MOTOR_CM_X = 0.23; // [m]

static const double RAW_MOTOR_POLY_0 = 57.0;
static const double RAW_MOTOR_POLY_1 = -9675.0;

struct BodyControl
{
    double f_z; // [N]
    double m_x; // [Nm]
    double m_y; // [Nm]
    double m_z; // [Nm]
};

typedef std::array<int16_t, N_MOTORS> MotorControl;

MotorControl body_to_motor_controls(BodyControl body_controls);

#endif /* MOTOR_CONTROL_H */
