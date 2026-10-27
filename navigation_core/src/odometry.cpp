#include "navigation/odometry.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

bool DifferentialOdometry::configure(const OdometryConfig& config) {
    if (!isFinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0f ||
        !isFinite(config.wheel_track_m) || config.wheel_track_m <= 0.0f) {
        return false;
    }
    config_ = config;
    return true;
}

void DifferentialOdometry::reset(const OdometryPose& initial_pose,
                                 float left_wheel_revolutions,
                                 float right_wheel_revolutions) {
    state_ = OdometryState{};
    state_.pose.x_m = isFinite(initial_pose.x_m) ? initial_pose.x_m : 0.0f;
    state_.pose.y_m = isFinite(initial_pose.y_m) ? initial_pose.y_m : 0.0f;
    state_.pose.heading_rad = normalizeAngle(initial_pose.heading_rad);
    previous_left_revolutions_ = isFinite(left_wheel_revolutions) ? left_wheel_revolutions : 0.0f;
    previous_right_revolutions_ = isFinite(right_wheel_revolutions) ? right_wheel_revolutions : 0.0f;
    initialized_ = true;
    state_.valid = true;
}

bool DifferentialOdometry::update(float left_wheel_revolutions,
                                  float right_wheel_revolutions, float dt_s) {
    if (!initialized_ || !isFinite(left_wheel_revolutions) ||
        !isFinite(right_wheel_revolutions) || !isFinite(dt_s) || dt_s <= 0.0f) {
        state_.valid = false;
        return false;
    }

    const float circumference = kTwoPi * config_.wheel_radius_m;
    const float dl = (left_wheel_revolutions - previous_left_revolutions_) * circumference;
    const float dr = (right_wheel_revolutions - previous_right_revolutions_) * circumference;
    previous_left_revolutions_ = left_wheel_revolutions;
    previous_right_revolutions_ = right_wheel_revolutions;

    const float ds = 0.5f * (dr + dl);
    const float dtheta = (dr - dl) / config_.wheel_track_m;
    const float midpoint_heading = state_.pose.heading_rad + 0.5f * dtheta;
    state_.pose.x_m += ds * std::cos(midpoint_heading);
    state_.pose.y_m += ds * std::sin(midpoint_heading);
    state_.pose.heading_rad = normalizeAngle(state_.pose.heading_rad + dtheta);
    state_.linear_velocity_mps = ds / dt_s;
    state_.angular_velocity_radps = dtheta / dt_s;
    state_.travelled_distance_m += std::fabs(ds);
    state_.valid = isFinite(state_.pose.x_m) && isFinite(state_.pose.y_m) &&
                   isFinite(state_.pose.heading_rad);
    return state_.valid;
}

}  // namespace nav
