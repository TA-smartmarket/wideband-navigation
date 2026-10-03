#include "navigation/differential_drive.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

WheelVelocities twistToWheelVelocities(const Twist& twist, float wheel_base_m) {
    WheelVelocities wheels;
    const float v = isFinite(twist.linear_mps) ? twist.linear_mps : 0.0f;
    const float omega = isFinite(twist.angular_radps) ? twist.angular_radps : 0.0f;
    const float half_base = 0.5f * wheel_base_m;
    wheels.left_mps = v - omega * half_base;
    wheels.right_mps = v + omega * half_base;
    return wheels;
}

Twist wheelVelocitiesToTwist(const WheelVelocities& wheels, float wheel_base_m) {
    Twist twist;
    if (!isFinite(wheel_base_m) || wheel_base_m <= 1.0e-6f) {
        return twist;  // degenerate chassis: no meaningful twist
    }
    twist.linear_mps = 0.5f * (wheels.left_mps + wheels.right_mps);
    twist.angular_radps = (wheels.right_mps - wheels.left_mps) / wheel_base_m;
    return twist;
}

float wheelAngularVelocity(float linear_velocity_mps, float wheel_radius_m) {
    if (!isFinite(wheel_radius_m) || wheel_radius_m <= 1.0e-6f) {
        return 0.0f;  // avoid division by zero
    }
    return linear_velocity_mps / wheel_radius_m;
}

MotorCommand wheelVelocitiesToCommands(const WheelVelocities& wheels,
                                       const DifferentialDriveConfig& config) {
    MotorCommand command;
    const float max_speed = config.max_wheel_speed_mps;
    if (!isFinite(max_speed) || max_speed <= 1.0e-6f) {
        return command;  // invalid configuration: stay stopped
    }

    const float left = isFinite(wheels.left_mps) ? wheels.left_mps : 0.0f;
    const float right = isFinite(wheels.right_mps) ? wheels.right_mps : 0.0f;

    command.left = clampf(left / max_speed, -1.0f, 1.0f);
    command.right = clampf(right / max_speed, -1.0f, 1.0f);

    // Per-side gain trim compensates a real motor/gearbox mismatch.
    command.left = clampf(command.left * (isFinite(config.left_gain) ? config.left_gain : 1.0f),
                          -1.0f, 1.0f);
    command.right = clampf(command.right * (isFinite(config.right_gain) ? config.right_gain : 1.0f),
                           -1.0f, 1.0f);

    // Deadband: commands that cannot overcome static friction are zeroed so the
    // controller does not sit in a buzzing half-on state.
    const float deadband = clampf(config.deadband_mps / max_speed, 0.0f, 0.5f);
    if (std::fabs(command.left) < deadband) {
        command.left = 0.0f;
    }
    if (std::fabs(command.right) < deadband) {
        command.right = 0.0f;
    }
    return command;
}

MotorCommand twistToMotorCommand(const Twist& twist, const DifferentialDriveConfig& config) {
    const WheelVelocities wheels = twistToWheelVelocities(twist, config.wheel_base_m);
    return wheelVelocitiesToCommands(wheels, config);
}

const char* validateDifferentialDriveConfig(const DifferentialDriveConfig& config) {
    if (!isFinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0f) {
        return "wheel_radius_m must be > 0";
    }
    if (!isFinite(config.wheel_base_m) || config.wheel_base_m <= 0.0f) {
        return "wheel_base_m must be > 0";
    }
    if (!isFinite(config.max_wheel_speed_mps) || config.max_wheel_speed_mps <= 0.0f) {
        return "max_wheel_speed_mps must be > 0";
    }
    if (!isFinite(config.left_gain) || config.left_gain <= 0.0f) {
        return "left_gain must be > 0";
    }
    if (!isFinite(config.right_gain) || config.right_gain <= 0.0f) {
        return "right_gain must be > 0";
    }
    return nullptr;
}

}  // namespace nav
