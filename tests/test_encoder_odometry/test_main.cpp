#include <unity.h>

#include <cmath>

#include "navigation/odometry.hpp"
#include "navigation/config_loader.hpp"
#include "navigation/path_follower.hpp"
#include "navigation/wheel_encoder.hpp"
#include "navigation/wheel_velocity_controller.hpp"

using namespace nav;

void setUp() {}
void tearDown() {}

static WheelEncoderConfig encoderConfig() {
    WheelEncoderConfig config;
    config.velocity_filter_alpha = 1.0f;
    config.max_abs_sensor_rpm = 1000.0f;
    return config;
}

static void test_encoder_unwraps_forward_and_reverse() {
    WheelEncoderTracker encoder;
    TEST_ASSERT_TRUE(encoder.configure(encoderConfig()));
    TEST_ASSERT_TRUE(encoder.update(4090, 1000000));
    TEST_ASSERT_TRUE(encoder.update(6, 1010000));
    TEST_ASSERT_EQUAL_INT64(12, encoder.sample().unwrapped_counts);
    TEST_ASSERT_TRUE(encoder.sample().raw_wheel_rpm > 0.0f);

    TEST_ASSERT_TRUE(encoder.update(4090, 1020000));
    TEST_ASSERT_EQUAL_INT64(0, encoder.sample().unwrapped_counts);
    TEST_ASSERT_TRUE(encoder.sample().raw_wheel_rpm < 0.0f);
}

static void test_encoder_applies_direction_and_ratio() {
    WheelEncoderConfig config = encoderConfig();
    config.invert_direction = true;
    config.sensor_to_wheel_ratio = 2.0f;
    WheelEncoderTracker encoder;
    TEST_ASSERT_TRUE(encoder.configure(config));
    TEST_ASSERT_TRUE(encoder.update(0, 1000000));
    TEST_ASSERT_TRUE(encoder.update(409, 2000000));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -409.0f / 4096.0f / 2.0f,
                             encoder.sample().wheel_revolutions);
}

static void test_encoder_fault_and_staleness() {
    WheelEncoderTracker encoder;
    TEST_ASSERT_TRUE(encoder.configure(encoderConfig()));
    TEST_ASSERT_TRUE(encoder.update(100, 1000000));
    TEST_ASSERT_TRUE(encoder.healthy(1050000));
    TEST_ASSERT_FALSE(encoder.healthy(1200000));
    encoder.markCommunicationFailure();
    TEST_ASSERT_FALSE(encoder.healthy(1050000));
    TEST_ASSERT_EQUAL_UINT32(1, encoder.communicationFailures());
}

static void test_odometry_straight_and_rotation() {
    DifferentialOdometry odometry;
    TEST_ASSERT_TRUE(odometry.configure(OdometryConfig{0.05f, 0.30f}));
    odometry.reset(OdometryPose{1.0f, 2.0f, 0.0f}, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(odometry.update(1.0f, 1.0f, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 1.0f + 2.0f * 3.14159265358979323846f * 0.05f,
                             odometry.state().pose.x_m);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 2.0f, odometry.state().pose.y_m);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.0f, odometry.state().pose.heading_rad);

    TEST_ASSERT_TRUE(odometry.update(0.5f, 1.5f, 1.0f));
    TEST_ASSERT_TRUE(std::fabs(odometry.state().pose.heading_rad) > 0.1f);
}

static void test_wheel_controller_stops_on_feedback_fault() {
    WheelVelocityControlConfig config;
    config.pid.kp = 0.5f;
    config.pid.ki = 0.1f;
    config.pid.kd = 0.0f;
    config.max_abs_rpm = 120.0f;
    config.tracking_error_rpm = 10.0f;
    config.tracking_error_timeout_ms = 100;
    WheelVelocityController controller;
    TEST_ASSERT_TRUE(controller.configure(config));
    TEST_ASSERT_TRUE(controller.update(60.0f, 0.0f, 0.01f, 10, true) > 60.0f);
    controller.update(60.0f, 0.0f, 0.01f, 120, true);
    TEST_ASSERT_TRUE(controller.status().tracking_fault);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,
                            controller.update(60.0f, 0.0f, 0.01f, 130, false));
    TEST_ASSERT_TRUE(controller.status().tracking_fault);
}

static void test_explicit_odometry_heading_is_accepted() {
    NavigationConfig config;
    PathFollower follower(config);
    follower.setMeasuredHeading(4.0f);
    TEST_ASSERT_TRUE(follower.headingValid());
    TEST_ASSERT_TRUE(follower.heading() <= 3.1415927f);
    TEST_ASSERT_TRUE(follower.heading() > -3.1415927f);
}

static void test_heading_pid_can_be_updated_safely() {
    NavigationConfig config;
    PathFollower follower(config);
    TEST_ASSERT_TRUE(follower.setHeadingPidGains(1.2f, 0.1f, 0.03f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, follower.config().heading_pid.kp);
    TEST_ASSERT_FALSE(follower.setHeadingPidGains(-1.0f, 0.0f, 0.0f));
}

static void test_closed_loop_configuration_is_loaded_and_validated() {
    const char* json = R"JSON({
      "position":{"source":"encoder_odometry"},
      "control":{"wheel_rate_hz":80},
      "encoders":{"common":{"counts_per_revolution":4096,
        "sensor_to_wheel_ratio":2.0,"velocity_filter_alpha":0.4,
        "max_abs_sensor_rpm":500,"stale_timeout_ms":120},
        "right":{"invert_direction":true}},
      "wheel_velocity_control":{"enabled":true,"common":{"kp":0.5,
        "ki":0.1,"kd":0.02,"max_abs_rpm":180,
        "tracking_error_timeout_ms":400}}
    })JSON";
    NavigationConfig config;
    char error[160] = {};
    TEST_ASSERT_TRUE(loadNavigationConfig(json, config, error, sizeof(error)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PositionSourceMode::ENCODER_ODOMETRY),
                          static_cast<int>(config.position_source));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 80.0f, config.control.wheel_rate_hz);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, config.encoder_left.sensor_to_wheel_ratio);
    TEST_ASSERT_TRUE(config.encoder_right.invert_direction);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, config.wheel_pid_left.pid.kp);

    TEST_ASSERT_FALSE(loadNavigationConfig(
        "{\"control\":{\"wheel_rate_hz\":5}}", config, error, sizeof(error)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_encoder_unwraps_forward_and_reverse);
    RUN_TEST(test_encoder_applies_direction_and_ratio);
    RUN_TEST(test_encoder_fault_and_staleness);
    RUN_TEST(test_odometry_straight_and_rotation);
    RUN_TEST(test_wheel_controller_stops_on_feedback_fault);
    RUN_TEST(test_explicit_odometry_heading_is_accepted);
    RUN_TEST(test_heading_pid_can_be_updated_safely);
    RUN_TEST(test_closed_loop_configuration_is_loaded_and_validated);
    return UNITY_END();
}
