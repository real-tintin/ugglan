#ifndef DRONE_PROPS_HPP
#define DRONE_PROPS_HPP

#include <cstdint>

namespace droneprops
{
inline const double I_XX = 0.014; // Moment of inertia about x [kgm^2].
inline const double I_YY = 0.014; // Moment of inertia about y [kgm^2].
inline const double I_ZZ = 0.026; // Moment of inertia about z [kgm^2].

inline const double TAU_MOTOR = 0.08; // Time constant of motors [s].

inline const uint8_t N_MOTORS = 4;
} // namespace droneprops

#endif /* DRONE_PROPS_HPP */
