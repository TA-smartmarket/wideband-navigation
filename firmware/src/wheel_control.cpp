#include "wheel_control.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace firmware {

namespace {
WheelControlSystem g_wheel_control;
}

float WheelControlSystem::clampCommand(float value) {
    if (!nav::isFinite(value)) return 0.0f;
    return nav::clampf(value, -1.0f, 1.0f);
}

bool WheelControlSystem::begin(const nav::NavigationConfig& config,
                               StepperMotorDriver* stepper) {
    config_ = config;
    stepper_ = stepper;
    if (!config.wheel_closed_loop_enabled || stepper == nullptr ||
        !encoders_.begin(config.encoder_left, config.encoder_right) ||
        !left_controller_.configure(config.wheel_pid_left) ||
        !right_controller_.configure(config.wheel_pid_right) ||
        !odometry_.configure(config.odometry)) {
        enabled_ = false;
        return false;
    }
    odometry_.reset(config.initial_pose, 0.0f, 0.0f);
    emergency_stop_latched_ = false;
    enabled_ = true;
    return true;
}

void WheelControlSystem::setNormalizedTarget(float left, float right) {
    portENTER_CRITICAL(&mux_);
    target_left_ = clampCommand(left);
    target_right_ = clampCommand(right);
    portEXIT_CRITICAL(&mux_);
}

void WheelControlSystem::stop() { setNormalizedTarget(0.0f, 0.0f); }

void WheelControlSystem::emergencyStop() {
    portENTER_CRITICAL(&mux_);
    target_left_ = 0.0f;
    target_right_ = 0.0f;
    emergency_stop_latched_ = true;
    portEXIT_CRITICAL(&mux_);
    left_controller_.reset();
    right_controller_.reset();
    if (stepper_ != nullptr) stepper_->emergencyStop();
}

void WheelControlSystem::clearEmergencyStop() {
    portENTER_CRITICAL(&mux_);
    target_left_ = 0.0f;
    target_right_ = 0.0f;
    emergency_stop_latched_ = false;
    portEXIT_CRITICAL(&mux_);
    left_controller_.reset();
    right_controller_.reset();
}

bool WheelControlSystem::requestPidUpdate(bool right_wheel, float kp, float ki, float kd) {
    if (!enabled_ || !nav::isFinite(kp) || !nav::isFinite(ki) || !nav::isFinite(kd) ||
        kp < 0.0f || kp > 10.0f || ki < 0.0f || ki > 10.0f || kd < 0.0f || kd > 10.0f) {
        return false;
    }
    portENTER_CRITICAL(&mux_);
    pending_pid_ = right_wheel ? config_.wheel_pid_right.pid : config_.wheel_pid_left.pid;
    pending_pid_.kp = kp;
    pending_pid_.ki = ki;
    pending_pid_.kd = kd;
    pending_pid_right_ = right_wheel;
    pending_pid_update_ = true;
    if (right_wheel) config_.wheel_pid_right.pid = pending_pid_;
    else config_.wheel_pid_left.pid = pending_pid_;
    portEXIT_CRITICAL(&mux_);
    return true;
}

bool WheelControlSystem::requestOdometryReset(const nav::OdometryPose& pose,
                                              bool zero_encoders) {
    if (!enabled_ || !nav::isFinite(pose.x_m) || !nav::isFinite(pose.y_m) ||
        !nav::isFinite(pose.heading_rad)) {
        return false;
    }
    portENTER_CRITICAL(&mux_);
    pending_pose_ = pose;
    pending_zero_encoders_ = zero_encoders;
    pending_odometry_reset_ = true;
    portEXIT_CRITICAL(&mux_);
    return true;
}

bool WheelControlSystem::recordHeadingPid(float kp, float ki, float kd) {
    if (!nav::isFinite(kp) || !nav::isFinite(ki) || !nav::isFinite(kd) ||
        kp < 0.0f || kp > 10.0f || ki < 0.0f || ki > 10.0f ||
        kd < 0.0f || kd > 10.0f) return false;
    portENTER_CRITICAL(&mux_);
    config_.heading_pid.kp = kp;
    config_.heading_pid.ki = ki;
    config_.heading_pid.kd = kd;
    config_.motion.heading_kp = kp;
    portEXIT_CRITICAL(&mux_);
    return true;
}

void WheelControlSystem::setPositionSource(nav::PositionSourceMode source) {
    portENTER_CRITICAL(&mux_);
    config_.position_source = source;
    portEXIT_CRITICAL(&mux_);
}

void WheelControlSystem::update(uint64_t now_us) {
    if (!enabled_ || stepper_ == nullptr) return;
    portENTER_CRITICAL(&mux_);
    const bool emergency_stop_latched = emergency_stop_latched_;
    portEXIT_CRITICAL(&mux_);
    if (emergency_stop_latched) {
        // The high-rate task must never re-enable the TB6600 after an E-stop.
        stepper_->emergencyStop();
        return;
    }
    nav::PIDConfig pending_pid;
    nav::WheelVelocityControlConfig pending_wheel_config;
    bool update_pid = false;
    bool update_right = false;
    bool reset_odometry = false;
    bool zero_encoders = false;
    nav::OdometryPose reset_pose;
    portENTER_CRITICAL(&mux_);
    if (pending_pid_update_) {
        pending_pid = pending_pid_;
        update_right = pending_pid_right_;
        pending_wheel_config = update_right ? config_.wheel_pid_right : config_.wheel_pid_left;
        pending_pid_update_ = false;
        update_pid = true;
    }
    if (pending_odometry_reset_) {
        reset_pose = pending_pose_;
        zero_encoders = pending_zero_encoders_;
        pending_odometry_reset_ = false;
        reset_odometry = true;
    }
    portEXIT_CRITICAL(&mux_);
    if (update_pid) {
        nav::WheelVelocityControlConfig wheel_config = pending_wheel_config;
        wheel_config.pid = pending_pid;
        if (update_right) {
            right_controller_.configure(wheel_config);
            // config_ was updated atomically when the request was accepted.
        } else {
            left_controller_.configure(wheel_config);
            // config_ was updated atomically when the request was accepted.
        }
    }
    if (reset_odometry) {
        stepper_->setRpm(0.0f, 0.0f);
        left_controller_.reset();
        right_controller_.reset();
        if (zero_encoders) {
            encoders_.resetTrackers();
            stepper_->leftAxis().zeroPosition();
            stepper_->rightAxis().zeroPosition();
            odometry_.reset(reset_pose, 0.0f, 0.0f);
        } else {
            odometry_.reset(reset_pose,
                            encoders_.left().sample().wheel_revolutions,
                            encoders_.right().sample().wheel_revolutions);
        }
    }
    const float nominal_dt = 1.0f / config_.control.wheel_rate_hz;
    float dt_s = previous_update_us_ == 0
                     ? nominal_dt
                     : static_cast<float>(now_us - previous_update_us_) * 1.0e-6f;
    previous_update_us_ = now_us;
    bool deadline_missed = false;
    if (!nav::isFinite(dt_s) || dt_s <= 0.0f || dt_s > nominal_dt * 4.0f) {
        dt_s = nominal_dt;
        deadline_missed = true;
    }

    encoders_.sample(now_us);
    const bool healthy = encoders_.left().healthy(now_us) &&
                         encoders_.right().healthy(now_us);
    float left_target = 0.0f;
    float right_target = 0.0f;
    portENTER_CRITICAL(&mux_);
    left_target = target_left_;
    right_target = target_right_;
    portEXIT_CRITICAL(&mux_);

    const float wheel_rpm_at_full_command =
        config_.drive.max_wheel_speed_mps * 60.0f /
        (nav::kTwoPi * config_.drive.wheel_radius_m);
    const float commanded_left_rpm = left_target * wheel_rpm_at_full_command;
    const float commanded_right_rpm = right_target * wheel_rpm_at_full_command;
    const auto& left_sample = encoders_.left().sample();
    const auto& right_sample = encoders_.right().sample();
    const float left_output = left_controller_.update(
        commanded_left_rpm, left_sample.filtered_wheel_rpm, dt_s, now_us / 1000u, healthy);
    const float right_output = right_controller_.update(
        commanded_right_rpm, right_sample.filtered_wheel_rpm, dt_s, now_us / 1000u, healthy);

    // Serialize the final hardware write with emergencyStop(). If E-stop wins
    // the lock, this cycle cannot re-enable the driver; if this cycle wins,
    // emergencyStop() runs immediately afterwards and leaves it disabled.
    portENTER_CRITICAL(&mux_);
    if (!emergency_stop_latched_) {
        if (!healthy || left_controller_.status().tracking_fault ||
            right_controller_.status().tracking_fault) {
            stepper_->setRpm(0.0f, 0.0f);
        } else {
            stepper_->setRpm(left_output, right_output);
        }
    }
    portEXIT_CRITICAL(&mux_);

    if (healthy) {
        odometry_.update(left_sample.wheel_revolutions,
                         right_sample.wheel_revolutions, dt_s);
    }

    portENTER_CRITICAL(&mux_);
    snapshot_.left = left_controller_.status();
    snapshot_.right = right_controller_.status();
    snapshot_.left_encoder = left_sample;
    snapshot_.right_encoder = right_sample;
    snapshot_.odometry = odometry_.state();
    snapshot_.left_step_count = stepper_->leftAxis().emittedSteps();
    snapshot_.right_step_count = stepper_->rightAxis().emittedSteps();
    snapshot_.encoder_healthy = healthy;
    if (deadline_missed) ++snapshot_.deadline_misses;
    ++snapshot_.update_count;
    portEXIT_CRITICAL(&mux_);
}

WheelControlSnapshot WheelControlSystem::snapshot() const {
    portENTER_CRITICAL(&mux_);
    const WheelControlSnapshot copy = snapshot_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

nav::NavigationConfig WheelControlSystem::configSnapshot() const {
    portENTER_CRITICAL(&mux_);
    const nav::NavigationConfig copy = config_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

bool WheelControlSystem::odometryMeasurement(nav::PositionMeasurement& measurement,
                                             uint64_t now_ms) const {
    const WheelControlSnapshot current = snapshot();
    if (!current.encoder_healthy || !current.odometry.valid) return false;
    measurement.timestamp_ms = now_ms;
    measurement.position.x_m = current.odometry.pose.x_m;
    measurement.position.y_m = current.odometry.pose.y_m;
    measurement.heading_rad = current.odometry.pose.heading_rad;
    measurement.heading_valid = true;
    measurement.quality = 0.70f;  // explicitly estimated, not global confidence
    measurement.valid = true;
    return true;
}

WheelControlSystem& wheelControl() { return g_wheel_control; }

}  // namespace firmware
