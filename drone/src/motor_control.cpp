#include "motor_control.h"

typedef std::array<double, N_MOTORS> AngularRate;

static AngularRate body_to_ang_rates(BodyControl body);
static MotorControl ang_rates_to_motor_controls(AngularRate ang_rates);

MotorControl body_to_motor_controls(BodyControl body_controls)
{
    AngularRate ang_rates = body_to_ang_rates(body_controls);
    MotorControl motor_controls = ang_rates_to_motor_controls(ang_rates);

    return motor_controls;
}

static AngularRate body_to_ang_rates(BodyControl body_controls)
{
    AngularRate ang_rates = {0.0, 0.0, 0.0, 0.0};
    AngularRate sq_ang_rates = {0.0, 0.0, 0.0, 0.0};

    double f_z = INV_THRUST_CONST_FZ * body_controls.f_z;
    double m_x = INV_THRUST_CONST_MZ * body_controls.m_x;
    double m_y = INV_THRUST_CONST_MZ * body_controls.m_y;
    double m_z = INV_TORQUE_CONST_MZ * body_controls.m_z;

    sq_ang_rates[0] = - f_z - m_x + m_y - m_z;
    sq_ang_rates[1] = - f_z - m_x - m_y + m_z;
    sq_ang_rates[2] = - f_z + m_x - m_y - m_z;
    sq_ang_rates[3] = - f_z + m_x + m_y + m_z;

    for (uint8_t motor_i = 0; motor_i < N_MOTORS; motor_i++)
    {
        if (sq_ang_rates[motor_i] > 0.0)
        {
            ang_rates[motor_i] = sqrt(sq_ang_rates[motor_i]);
        }
    }

    return ang_rates;
}

static MotorControl ang_rates_to_motor_controls(AngularRate ang_rates)
{
    double motor_control = 0.0;
    MotorControl motor_controls = {0, 0, 0, 0};

    for (uint8_t motor_i = 0; motor_i < N_MOTORS; motor_i++)
    {
        motor_control = RAW_MOTOR_POLY_0 * ang_rates[motor_i] + RAW_MOTOR_POLY_1;

        if (motor_control < 0.0)
        {
            motor_control = 0.0;
        }
        if (motor_control > INT16_MAX)
        {
            motor_control = INT16_MAX;
        }

        motor_controls[motor_i] = int16_t(motor_control);
    }

    return motor_controls;
}
