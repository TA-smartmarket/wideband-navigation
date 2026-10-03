// Differential drive kinematics: converts a body twist (v, omega) into left
// and right wheel linear velocities and normalised motor commands.
#pragma once

#include "navigation/types.hpp"

namespace nav {

struct DifferentialDriveConfig {
    float wheel_radius_m{0.05f};
    float wheel_base_m{0.32f};
    float max_wheel_speed_mps{0.70f};
    /// Speed below which a motor is commanded to zero (compensates static friction).
    float deadband_mps{0.005f};
    float left_gain{1.0f};
    float right_gain{1.0f};
};

struct WheelVelocities {
    float left_mps{0.0f};
    float right_mps{0.0f};
};

struct Twist {
    float linear_mps{0.0f};
    float angular_radps{0.0f};
};

/// Inverse kinematics: (v, omega) -> wheel linear velocities.
WheelVelocities twistToWheelVelocities(const Twist& twist, float wheel_base_m);

/// Forward kinematics: wheel linear velocities -> (v, omega).
Twist wheelVelocitiesToTwist(const WheelVelocities& wheels, float wheel_base_m);

/// Wheel angular velocity (rad/s) from linear velocity and wheel radius.
float wheelAngularVelocity(float linear_velocity_mps, float wheel_radius_m);

/// Convert wheel linear velocities into normalised commands in [-1, +1].
MotorCommand wheelVelocitiesToCommands(const WheelVelocities& wheels,
                                       const DifferentialDriveConfig& config);

/// Full conversion used by the path follower: twist -> normalised commands.
MotorCommand twistToMotorCommand(const Twist& twist, const DifferentialDriveConfig& config);

/// Validate the physical parameters; returns nullptr when acceptable, else a
/// human readable reason (used at start-up).
const char* validateDifferentialDriveConfig(const DifferentialDriveConfig& config);

}  // namespace nav
