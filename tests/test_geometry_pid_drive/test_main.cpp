// Unit tests: geometry helpers, PID controller and differential drive maths.
#include <unity.h>

#include <cmath>

#include "navigation/differential_drive.hpp"
#include "navigation/geometry.hpp"
#include "navigation/pid.hpp"

using namespace nav;

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

static void test_euclidean_distance() {
    TEST_ASSERT_EQUAL_FLOAT(5.0f, euclideanDistance(Position2D{0.0f, 0.0f}, Position2D{3.0f, 4.0f}));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, euclideanDistance(Position2D{1.0f, 1.0f}, Position2D{1.0f, 1.0f}));
    TEST_ASSERT_EQUAL_FLOAT(25.0f,
                            euclideanDistanceSquared(Position2D{0.0f, 0.0f}, Position2D{3.0f, 4.0f}));
    // Non-finite input must not propagate NaN.
    TEST_ASSERT_TRUE(isFinite(euclideanDistance(Position2D{NAN, 0.0f}, Position2D{1.0f, 1.0f})));
}

static void test_angle_normalization() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, normalizeAngle(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, normalizeAngle(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, normalizeAngle(-0.5f));
    // +3*pi/2 wraps to -pi/2.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -kPi / 2.0f, normalizeAngle(3.0f * kPi / 2.0f));
    // -3*pi/2 wraps to +pi/2.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, kPi / 2.0f, normalizeAngle(-3.0f * kPi / 2.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, normalizeAngle(kTwoPi));
    // A large value still lands inside [-pi, pi].
    const float wrapped = normalizeAngle(100.0f);
    TEST_ASSERT_TRUE(wrapped >= -kPi - 1e-4f && wrapped <= kPi + 1e-4f);
    // Non-finite input is neutralised.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, normalizeAngle(NAN));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, normalizeAngle(INFINITY));
}

static void test_shortest_angular_distance() {
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, shortestAngularDistance(0.0f, 0.1f));
    // 3.0 -> -3.0: the direct difference is -6.0 rad, so the short way is
    // +2*pi - 6.0 = +0.283 rad (the test values cross the +/-pi branch cut).
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, kTwoPi - 6.0f, shortestAngularDistance(3.0f, -3.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -(kTwoPi - 6.0f), shortestAngularDistance(-3.0f, 3.0f));
    // Symmetry: the reverse rotation is the negated forward rotation.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.4f, shortestAngularDistance(0.2f, -0.2f));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, shortestAngularDistance(-0.2f, 0.2f));
}

static void test_bearing_calculation() {
    const Position2D origin{0.0f, 0.0f};
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, calculateBearing(origin, Position2D{1.0f, 0.0f}));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kPi / 2.0f, calculateBearing(origin, Position2D{0.0f, 1.0f}));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -kPi / 2.0f, calculateBearing(origin, Position2D{0.0f, -1.0f}));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kPi, std::fabs(calculateBearing(origin, Position2D{-1.0f, 0.0f})));
    // Coincident points: no direction, returns 0 instead of NaN.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, calculateBearing(origin, origin));
}

static void test_point_to_segment_distance() {
    const Position2D a{0.0f, 0.0f};
    const Position2D b{4.0f, 0.0f};
    // Perpendicular projection inside the segment.
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pointToSegmentDistance(Position2D{2.0f, 2.0f}, a, b));
    // Beyond the end: clamps to the endpoint.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pointToSegmentDistance(Position2D{5.0f, 0.0f}, a, b));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pointToSegmentDistance(Position2D{-2.0f, 0.0f}, a, b));
    // Degenerate segment falls back to point-to-point.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pointToSegmentDistance(Position2D{1.0f, 0.0f}, a, a));
    const Position2D closest = closestPointOnSegment(Position2D{2.0f, 2.0f}, a, b);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, closest.x_m);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, closest.y_m);
}

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------

static void test_pid_proportional_only() {
    PIDConfig config;
    config.kp = 2.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.output_min = -10.0f;
    config.output_max = 10.0f;
    PIDController pid(config);
    // setpoint 5, measurement 3 => error 2 => output 4
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, pid.update(5.0f, 3.0f, 0.05f));
}

static void test_pid_saturation() {
    PIDConfig config;
    config.kp = 100.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.output_min = -1.5f;
    config.output_max = 1.5f;
    PIDController pid(config);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, pid.update(10.0f, 0.0f, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(-1.5f, pid.update(-10.0f, 0.0f, 0.05f));
}

static void test_pid_integral_accumulates_and_saturates() {
    PIDConfig config;
    config.kp = 0.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    config.integral_min = -0.5f;
    config.integral_max = 0.5f;
    PIDController pid(config);
    // error 1 * ki 1 * dt 0.5 = +0.5 per step, clamped by integral_max.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, pid.update(1.0f, 0.0f, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, pid.update(1.0f, 0.0f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.integralTerm());
}

static void test_pid_anti_windup_limits_integral_while_saturated() {
    PIDConfig config;
    config.kp = 0.0f;
    config.ki = 10.0f;
    config.kd = 0.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    config.integral_min = -100.0f;
    config.integral_max = 100.0f;
    PIDController pid(config);
    // Step 1 saturates the output.  Back-calculation rewrites the integrator to
    // output - (p + d) = 1.0 instead of letting it climb to +5.0 per step.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(1.0f, 0.0f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.integralTerm());
    // Further steps with the same error keep the integrator pinned: no windup.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(1.0f, 0.0f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.integralTerm());
    // Reversing the error must unwind immediately (no stored integral lag).
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.update(-1.0f, 0.0f, 0.5f));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, pid.integralTerm());
}

static void test_pid_integral_is_usable_after_recovery() {
    // Once the error shrinks, the integral accumulated during saturation must
    // still be available to hold the actuator at the operating point.
    PIDConfig config;
    config.kp = 1.0f;
    config.ki = 4.0f;
    config.kd = 0.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    config.integral_min = -2.0f;
    config.integral_max = 2.0f;
    PIDController pid(config);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.update(1.0f, 0.0f, 0.25f));
    // Small residual error now: output stays smooth, no discontinuity.
    const float output = pid.update(0.2f, 0.0f, 0.25f);
    TEST_ASSERT_TRUE(isFinite(output));
    TEST_ASSERT_TRUE(output <= 1.0f && output >= -1.0f);
}

static void test_pid_reset_clears_state() {
    PIDConfig config;
    config.kp = 1.0f;
    config.ki = 1.0f;
    config.kd = 0.0f;
    PIDController pid(config);
    pid.update(1.0f, 0.0f, 0.5f);
    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integralTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.derivativeTerm());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, pid.update(1.0f, 0.0f, 0.5f));
}

static void test_pid_derivative_responds_to_measurement_change() {
    PIDConfig config;
    config.kp = 0.0f;
    config.ki = 0.0f;
    config.kd = 1.0f;
    config.derivative_filter_alpha = 1.0f;  // no filtering, makes the value exact
    config.output_min = -50.0f;             // wide enough to observe the raw term
    config.output_max = 50.0f;
    PIDController pid(config);
    pid.update(0.0f, 0.0f, 0.1f);  // establishes the previous measurement
    // measurement moved +1.0 in 0.1 s => derivative term = -10
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -10.0f, pid.update(0.0f, 1.0f, 0.1f));
}

static void test_pid_handles_degenerate_dt_and_nan() {
    PIDConfig config;
    config.kp = 1.0f;
    config.ki = 1.0f;
    config.kd = 1.0f;
    config.nominal_dt_s = 0.05f;
    PIDController pid(config);
    // dt = 0 must not divide by zero.
    TEST_ASSERT_TRUE(isFinite(pid.update(1.0f, 0.0f, 0.0f)));
    // dt = 1e6 s is treated as the nominal sample time.
    TEST_ASSERT_TRUE(isFinite(pid.update(1.0f, 0.0f, 1.0e6f)));
    // A NaN measurement must not poison the output.
    const float out = pid.update(1.0f, NAN, 0.05f);
    TEST_ASSERT_TRUE(isFinite(out));
    TEST_ASSERT_TRUE(isFinite(pid.update(NAN, 0.0f, 0.05f)));
}

// ---------------------------------------------------------------------------
// Differential drive
// ---------------------------------------------------------------------------

static void test_differential_drive_inverse_kinematics() {
    // v = 0.4 m/s straight: both wheels at 0.4 m/s.
    const WheelVelocities straight = twistToWheelVelocities(Twist{0.4f, 0.0f}, 0.32f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, straight.left_mps);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, straight.right_mps);

    // omega = 1.0 rad/s, v = 0: left = -0.16, right = +0.16.
    const WheelVelocities spin = twistToWheelVelocities(Twist{0.0f, 1.0f}, 0.32f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.16f, spin.left_mps);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f, spin.right_mps);
}

static void test_differential_drive_forward_kinematics_round_trip() {
    const WheelVelocities wheels{0.2f, 0.6f};
    const Twist twist = wheelVelocitiesToTwist(wheels, 0.32f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, twist.linear_mps);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.25f, twist.angular_radps);
    // Degenerate wheel base must not divide by zero.
    const Twist degenerate = wheelVelocitiesToTwist(wheels, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, degenerate.angular_radps);
}

static void test_wheel_angular_velocity() {
    // 0.4 m/s with a 0.05 m radius => 8 rad/s.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.0f, wheelAngularVelocity(0.4f, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, wheelAngularVelocity(0.4f, 0.0f));
}

static void test_normalised_motor_commands() {
    DifferentialDriveConfig config;
    config.wheel_base_m = 0.32f;
    config.max_wheel_speed_mps = 0.60f;
    config.deadband_mps = 0.0f;
    const MotorCommand command = twistToMotorCommand(Twist{0.3f, 1.0f}, config);
    // left = 0.3 - 0.16 = 0.14 => 0.2333; right = 0.46 => 0.7667
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.14f / 0.60f, command.left);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.46f / 0.60f, command.right);
    TEST_ASSERT_TRUE(command.left >= -1.0f && command.left <= 1.0f);
    TEST_ASSERT_TRUE(command.right >= -1.0f && command.right <= 1.0f);
}

static void test_motor_commands_are_saturated() {
    DifferentialDriveConfig config;
    config.max_wheel_speed_mps = 0.10f;
    config.deadband_mps = 0.0f;
    const MotorCommand command = twistToMotorCommand(Twist{5.0f, 0.0f}, config);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, command.left);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, command.right);
}

static void test_motor_deadband_and_gain_trim() {
    DifferentialDriveConfig config;
    config.max_wheel_speed_mps = 0.60f;
    config.deadband_mps = 0.05f;  // 0.05/0.60 = 8.3% deadband
    config.left_gain = 1.0f;
    config.right_gain = 0.9f;
    const MotorCommand command = twistToMotorCommand(Twist{0.02f, 0.0f}, config);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, command.right);

    const MotorCommand trimmed = twistToMotorCommand(Twist{0.60f, 0.0f}, config);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, trimmed.left);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.9f, trimmed.right);
}

static void test_drive_config_validation() {
    DifferentialDriveConfig config;
    TEST_ASSERT_NULL(validateDifferentialDriveConfig(config));
    config.wheel_radius_m = 0.0f;
    TEST_ASSERT_TRUE(validateDifferentialDriveConfig(config) != nullptr);
    config.wheel_radius_m = 0.05f;
    config.wheel_base_m = 0.0f;
    TEST_ASSERT_TRUE(validateDifferentialDriveConfig(config) != nullptr);
    config.wheel_base_m = 0.32f;
    config.max_wheel_speed_mps = 0.0f;
    TEST_ASSERT_TRUE(validateDifferentialDriveConfig(config) != nullptr);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_euclidean_distance);
    RUN_TEST(test_angle_normalization);
    RUN_TEST(test_shortest_angular_distance);
    RUN_TEST(test_bearing_calculation);
    RUN_TEST(test_point_to_segment_distance);
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_saturation);
    RUN_TEST(test_pid_integral_accumulates_and_saturates);
    RUN_TEST(test_pid_anti_windup_limits_integral_while_saturated);
    RUN_TEST(test_pid_integral_is_usable_after_recovery);
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_derivative_responds_to_measurement_change);
    RUN_TEST(test_pid_handles_degenerate_dt_and_nan);
    RUN_TEST(test_differential_drive_inverse_kinematics);
    RUN_TEST(test_differential_drive_forward_kinematics_round_trip);
    RUN_TEST(test_wheel_angular_velocity);
    RUN_TEST(test_normalised_motor_commands);
    RUN_TEST(test_motor_commands_are_saturated);
    RUN_TEST(test_motor_deadband_and_gain_trim);
    RUN_TEST(test_drive_config_validation);
    return UNITY_END();
}
