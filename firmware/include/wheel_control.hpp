#pragma once

#include <Arduino.h>

#include "as5600_encoder.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/odometry.hpp"
#include "navigation/wheel_velocity_controller.hpp"
#include "stepper_driver.hpp"

namespace firmware {

struct WheelControlSnapshot {
    nav::WheelVelocityControlStatus left{};
    nav::WheelVelocityControlStatus right{};
    nav::WheelEncoderSample left_encoder{};
    nav::WheelEncoderSample right_encoder{};
    nav::OdometryState odometry{};
    uint64_t left_step_count{0};
    uint64_t right_step_count{0};
    uint32_t update_count{0};
    uint32_t deadline_misses{0};
    bool encoder_healthy{false};
};

class WheelControlSystem {
public:
    bool begin(const nav::NavigationConfig& config, StepperMotorDriver* stepper);
    void setNormalizedTarget(float left, float right);
    void stop();
    void emergencyStop();
    void clearEmergencyStop();
    bool requestPidUpdate(bool right_wheel, float kp, float ki, float kd);
    bool recordHeadingPid(float kp, float ki, float kd);
    bool requestOdometryReset(const nav::OdometryPose& pose, bool zero_encoders);
    void setPositionSource(nav::PositionSourceMode source);
    void update(uint64_t now_us);
    WheelControlSnapshot snapshot() const;
    nav::NavigationConfig configSnapshot() const;
    bool odometryMeasurement(nav::PositionMeasurement& measurement, uint64_t now_ms) const;
    bool enabled() const { return enabled_; }

private:
    static float clampCommand(float value);

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    nav::NavigationConfig config_{};
    StepperMotorDriver* stepper_{nullptr};
    DualAs5600 encoders_{};
    nav::WheelVelocityController left_controller_{};
    nav::WheelVelocityController right_controller_{};
    nav::DifferentialOdometry odometry_{};
    WheelControlSnapshot snapshot_{};
    float target_left_{0.0f};
    float target_right_{0.0f};
    uint64_t previous_update_us_{0};
    bool enabled_{false};
    nav::PIDConfig pending_pid_{};
    bool pending_pid_right_{false};
    bool pending_pid_update_{false};
    nav::OdometryPose pending_pose_{};
    bool pending_zero_encoders_{false};
    bool pending_odometry_reset_{false};
    bool emergency_stop_latched_{false};
};

WheelControlSystem& wheelControl();

}  // namespace firmware
