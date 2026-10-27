// Differential-drive odometry from measured wheel revolutions.
#pragma once

#include "navigation/types.hpp"

namespace nav {

struct OdometryConfig {
    float wheel_radius_m{0.05f};
    float wheel_track_m{0.32f};
};

struct OdometryPose {
    float x_m{0.0f};
    float y_m{0.0f};
    float heading_rad{0.0f};
};

struct OdometryState {
    OdometryPose pose{};
    float linear_velocity_mps{0.0f};
    float angular_velocity_radps{0.0f};
    float travelled_distance_m{0.0f};
    bool valid{false};
};

class DifferentialOdometry {
public:
    bool configure(const OdometryConfig& config);
    void reset(const OdometryPose& initial_pose, float left_wheel_revolutions,
               float right_wheel_revolutions);
    bool update(float left_wheel_revolutions, float right_wheel_revolutions,
                float dt_s);

    const OdometryState& state() const { return state_; }

private:
    OdometryConfig config_{};
    OdometryState state_{};
    float previous_left_revolutions_{0.0f};
    float previous_right_revolutions_{0.0f};
    bool initialized_{false};
};

}  // namespace nav
