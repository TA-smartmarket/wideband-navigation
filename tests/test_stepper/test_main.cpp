// Unit tests: stepper control (NEMA 17 + A4988 class).
//
// A step/dir driver is not a PWM device: speed is the STEP frequency, direction
// is a logic level and position is the pulse count.  These tests pin that
// conversion, the trapezoidal position profile and the direction handling, which
// is what makes the stepper path behave correctly without any hardware present.
#include <unity.h>

#include <cmath>

#include "navigation/stepper.hpp"

using namespace nav;

namespace {

StepperConfig defaultConfig() {
    StepperConfig config;
    config.steps_per_revolution = 200;  // NEMA 17
    config.microsteps = 16;             // A4988 at 1/16
    config.wheel_radius_m = 0.05f;
    config.max_step_rate_hz = 20000.0f;
    config.min_step_rate_hz = 2.0f;
    config.max_step_accel_hz_per_s = 20000.0f;
    return config;
}

/// Circumference of the wheel, in meters.
constexpr float kWheelCircumference = 2.0f * kPi * 0.05f;

}  // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Conversion maths
// ---------------------------------------------------------------------------

static void test_steps_per_revolution_includes_microstepping() {
    StepperConfig config = defaultConfig();
    TEST_ASSERT_EQUAL_INT(3200, stepsPerRevolution(config));  // 200 * 16

    config.microsteps = 1;  // full step
    TEST_ASSERT_EQUAL_INT(200, stepsPerRevolution(config));

    config.microsteps = 0;  // invalid -> treated as full step
    TEST_ASSERT_EQUAL_INT(200, stepsPerRevolution(config));
}

static void test_steps_per_meter() {
    const StepperConfig config = defaultConfig();
    // 3200 steps per revolution over a 0.314159 m circumference.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 3200.0f / kWheelCircumference, stepsPerMeter(config));
}

static void test_velocity_and_rate_are_inverse() {
    const StepperConfig config = defaultConfig();
    for (float velocity : {0.05f, 0.2f, 0.45f, 0.7f}) {
        const float rate = wheelVelocityToStepRate(velocity, config);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, velocity, stepRateToWheelVelocity(rate, config));
    }
    // 0.45 m/s on a 0.05 m radius wheel is about 85.9 RPM.
    const float rpm = stepRateToRpm(wheelVelocityToStepRate(0.45f, config), config);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 85.94f, rpm);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.45f,
                             stepRateToWheelVelocity(rpmToStepRate(rpm, config), config));
}

static void test_steps_to_distance_and_revolutions() {
    const StepperConfig config = defaultConfig();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, stepsToRevolutions(3200, config));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kWheelCircumference, stepsToDistanceM(3200, config));
    // One wheel revolution must equal one circumference of ground travel.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, stepsToDistanceM(stepsPerRevolution(config), config),
                             kWheelCircumference);
}

static void test_configuration_validation() {
    StepperConfig config = defaultConfig();
    TEST_ASSERT_TRUE(validateStepperConfig(config).valid);

    config.steps_per_revolution = 0;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
    config = defaultConfig();
    config.microsteps = 0;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
    config = defaultConfig();
    config.wheel_radius_m = 0.0f;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
    config = defaultConfig();
    config.max_step_rate_hz = 0.0f;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
    config = defaultConfig();
    config.max_step_accel_hz_per_s = 0.0f;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
    config = defaultConfig();
    config.min_step_rate_hz = config.max_step_rate_hz + 1.0f;
    TEST_ASSERT_FALSE(validateStepperConfig(config).valid);
}

// ---------------------------------------------------------------------------
// Continuous (velocity) mode
// ---------------------------------------------------------------------------

static void test_continuous_mode_reaches_commanded_rate_and_counts_steps() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetRateHz(3200.0f);  // exactly one revolution per second

    // The acceleration ramp means the rate is not reached instantly.
    int32_t first_cycle = axis.advance(0.01f);
    TEST_ASSERT_TRUE(first_cycle > 0);
    TEST_ASSERT_TRUE(axis.currentRateHz() < 3200.0f);

    // Run long enough to settle at the commanded rate.
    for (int i = 0; i < 200; ++i) {
        axis.advance(0.01f);
    }
    TEST_ASSERT_TRUE(axis.atTargetRate());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 3200.0f, axis.currentRateHz());
    // 3200 steps/s over 3200 steps/rev is exactly 1 rev/s, i.e. 60 RPM.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 60.0f, axis.currentRpm());
    // At a steady 1 rev/s the ground speed is one circumference per second.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kWheelCircumference,
                             stepRateToWheelVelocity(axis.currentRateHz(), axis.config()));

}

static void test_step_count_matches_commanded_distance() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetRateHz(3200.0f);
    // Settle the ramp first, then measure a known window.
    for (int i = 0; i < 300; ++i) {
        axis.advance(0.01f);
    }
    axis.zeroPosition();
    const int64_t before = axis.positionSteps();
    float elapsed = 0.0f;
    const int32_t steps = [&] {
        int32_t total = 0;
        while (elapsed < 1.0f - 1e-6f) {
            total += axis.advance(0.01f);
            elapsed += 0.01f;
        }
        return total;
    }();
    // 3200 steps/s for 1 s, so the count must be close to one revolution.
    TEST_ASSERT_INT_WITHIN(8, 3200, steps);
    TEST_ASSERT_EQUAL_INT64(before + steps, axis.positionSteps());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, kWheelCircumference, axis.distanceM());
}

static void test_direction_follows_the_sign_of_the_command() {
    StepperAxis axis;
    axis.configure(defaultConfig());

    axis.setTargetRateHz(3200.0f);
    for (int i = 0; i < 50; ++i) axis.advance(0.01f);
    TEST_ASSERT_EQUAL_INT(1, axis.direction());
    TEST_ASSERT_TRUE(axis.positionSteps() > 0);

    axis.setTargetRateHz(-3200.0f);
    for (int i = 0; i < 50; ++i) axis.advance(0.01f);
    TEST_ASSERT_EQUAL_INT(-1, axis.direction());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -3200.0f, axis.currentRateHz());

    // Zero command stops the axis and clears the direction.
    axis.setTargetRateHz(0.0f);
    for (int i = 0; i < 100; ++i) axis.advance(0.01f);
    TEST_ASSERT_EQUAL_INT(0, axis.direction());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, axis.currentRateHz());
    TEST_ASSERT_FALSE(axis.moving());
}

static void test_inverted_direction_flips_only_the_dir_level() {
    StepperConfig config = defaultConfig();
    config.invert_direction = true;
    StepperAxis axis;
    axis.configure(config);
    axis.setTargetRateHz(3200.0f);
    for (int i = 0; i < 50; ++i) axis.advance(0.01f);
    // The pulse rate is positive, but the DIR level the driver sees is mirrored.
    TEST_ASSERT_TRUE(axis.currentRateHz() > 0.0f);
    TEST_ASSERT_EQUAL_INT(-1, axis.direction());
    // Position still advances in the commanded direction.
    TEST_ASSERT_TRUE(axis.positionSteps() > 0);
}

static void test_rate_is_clamped_and_tiny_commands_stop() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetRateHz(1.0e9f);  // absurd
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 20000.0f, axis.targetRateHz());
    // A command below the minimum step rate is treated as stop, so the driver
    // cannot emit stray single pulses.
    axis.setTargetRateHz(0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, axis.targetRateHz());
}

static void test_zero_and_negative_dt_emit_nothing() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetRateHz(3200.0f);
    TEST_ASSERT_EQUAL_INT(0, axis.advance(0.0f));
    TEST_ASSERT_EQUAL_INT(0, axis.advance(-0.01f));
    TEST_ASSERT_EQUAL_INT(0, axis.advance(NAN));
}

static void test_fractional_rates_accumulate_without_bias() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    // A rate that is not an integer multiple of the cycle rate: 333.3 Hz at
    // 10 ms cycles = 3.333 steps per cycle.  Over 1 s the count must be ~333,
    // not 3 per cycle (which would be 300).
    axis.setTargetRateHz(333.0f);
    for (int i = 0; i < 400; ++i) axis.advance(0.01f);  // settle the ramp
    axis.zeroPosition();
    for (int i = 0; i < 100; ++i) axis.advance(0.01f);
    TEST_ASSERT_INT_WITHIN(2, 333, axis.positionSteps());
}

// ---------------------------------------------------------------------------
// Position (move) mode
// ---------------------------------------------------------------------------

static void test_position_mode_stops_exactly_on_target() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.moveRevolutions(2.0f);  // 6400 steps
    TEST_ASSERT_EQUAL_INT64(6400, axis.targetPositionSteps());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(StepperMode::POSITION),
                          static_cast<int>(axis.mode()));

    int guard = 0;
    while (!axis.positionReached() && guard++ < 20000) {
        axis.advance(0.002f);
    }
    TEST_ASSERT_TRUE(axis.positionReached());
    // No overshoot and no hunting: exactly on target with the rate back to zero.
    TEST_ASSERT_EQUAL_INT64(6400, axis.positionSteps());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, axis.currentRateHz());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f, axis.revolutions());
}

static void test_position_mode_decelerates_before_the_target() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetPositionSteps(20000);
    float peak = 0.0f;
    int guard = 0;
    while (!axis.positionReached() && guard++ < 200000) {
        // The braking limit is evaluated against the distance available at the
        // START of the cycle, because that is what the axis can act on.  Checking
        // against the post-step remainder would be stricter than any discrete
        // controller can satisfy (it would require zero travel per cycle).
        const int64_t remaining_before = axis.targetPositionSteps() - axis.positionSteps();
        const float limit = std::sqrt(2.0f * axis.config().max_step_accel_hz_per_s *
                                      static_cast<float>(remaining_before));
        axis.advance(0.001f);
        peak = std::fmax(peak, std::fabs(axis.currentRateHz()));
        // The rate must never exceed what the remaining distance allows the axis
        // to brake from, otherwise the move would overshoot.
        TEST_ASSERT_TRUE(std::fabs(axis.currentRateHz()) <= limit + 1.0f);
    }
    TEST_ASSERT_TRUE(axis.positionReached());
    TEST_ASSERT_EQUAL_INT64(20000, axis.positionSteps());
    TEST_ASSERT_TRUE(peak > 1000.0f);  // it really did accelerate
}

static void test_position_mode_handles_negative_moves() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetPositionSteps(-3200);  // one revolution backwards
    int guard = 0;
    while (!axis.positionReached() && guard++ < 20000) {
        axis.advance(0.002f);
    }
    TEST_ASSERT_TRUE(axis.positionReached());
    TEST_ASSERT_EQUAL_INT64(-3200, axis.positionSteps());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.0f, axis.revolutions());
}

static void test_switching_back_to_continuous_mode() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.moveRevolutions(1.0f);
    axis.advance(0.002f);
    axis.setContinuousMode();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(StepperMode::CONTINUOUS),
                          static_cast<int>(axis.mode()));
    axis.setTargetRateHz(1000.0f);
    for (int i = 0; i < 100; ++i) axis.advance(0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, axis.currentRateHz());
}

// ---------------------------------------------------------------------------
// Differential drive
// ---------------------------------------------------------------------------

static void test_differential_stepper_drive_straight_and_turn() {
    StepperDrive drive;
    drive.configure(defaultConfig(), defaultConfig(), 0.32f, 0.70f);
    drive.reset();

    // Straight: both axes must run at the same rate and travel the same distance.
    for (int i = 0; i < 100; ++i) {
        drive.commandTwist(0.30f, 0.0f, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0f, drive.left().currentRateHz(), drive.right().currentRateHz());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drive.odometryYawRad());

    // Spin in place: equal and opposite wheel travel, so the odometry distance
    // stays (near) zero while the yaw changes - the property that makes
    // wheel-odometry distance a *translational* measure.
    drive.reset();
    const float distance_before = drive.odometryDistanceM();
    for (int i = 0; i < 200; ++i) {
        drive.commandTwist(0.0f, 1.0f, 0.01f);
    }
    const float translation = std::fabs(drive.odometryDistanceM() - distance_before);
    TEST_ASSERT_TRUE(translation < 0.01f);
    TEST_ASSERT_TRUE(std::fabs(drive.odometryYawRad()) > 0.1f);
    TEST_ASSERT_TRUE(drive.left().currentRateHz() < 0.0f);
    TEST_ASSERT_TRUE(drive.right().currentRateHz() > 0.0f);
}

static void test_drive_clamps_wheel_speed() {
    StepperDrive drive;
    drive.configure(defaultConfig(), defaultConfig(), 0.32f, 0.70f);
    for (int i = 0; i < 400; ++i) {
        drive.commandTwist(5.0f, 0.0f, 0.01f);  // absurd request
    }
    // 0.70 m/s ceiling -> 0.70 * 10185.9 steps/m, never more.
    const float max_rate = wheelVelocityToStepRate(0.70f, defaultConfig());
    TEST_ASSERT_TRUE(drive.left().currentRateHz() <= max_rate + 1.0f);
    TEST_ASSERT_TRUE(drive.right().currentRateHz() <= max_rate + 1.0f);
}

static void test_reset_clears_position_and_rate() {
    StepperAxis axis;
    axis.configure(defaultConfig());
    axis.setTargetRateHz(3200.0f);
    for (int i = 0; i < 50; ++i) axis.advance(0.01f);
    TEST_ASSERT_TRUE(axis.positionSteps() != 0);
    axis.reset();
    TEST_ASSERT_EQUAL_INT64(0, axis.positionSteps());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, axis.currentRateHz());
    TEST_ASSERT_EQUAL_UINT64(0, axis.emittedSteps());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_steps_per_revolution_includes_microstepping);
    RUN_TEST(test_steps_per_meter);
    RUN_TEST(test_velocity_and_rate_are_inverse);
    RUN_TEST(test_steps_to_distance_and_revolutions);
    RUN_TEST(test_configuration_validation);
    RUN_TEST(test_continuous_mode_reaches_commanded_rate_and_counts_steps);
    RUN_TEST(test_step_count_matches_commanded_distance);
    RUN_TEST(test_direction_follows_the_sign_of_the_command);
    RUN_TEST(test_inverted_direction_flips_only_the_dir_level);
    RUN_TEST(test_rate_is_clamped_and_tiny_commands_stop);
    RUN_TEST(test_zero_and_negative_dt_emit_nothing);
    RUN_TEST(test_fractional_rates_accumulate_without_bias);
    RUN_TEST(test_position_mode_stops_exactly_on_target);
    RUN_TEST(test_position_mode_decelerates_before_the_target);
    RUN_TEST(test_position_mode_handles_negative_moves);
    RUN_TEST(test_switching_back_to_continuous_mode);
    RUN_TEST(test_differential_stepper_drive_straight_and_turn);
    RUN_TEST(test_drive_clamps_wheel_speed);
    RUN_TEST(test_reset_clears_position_and_rate);
    return UNITY_END();
}
