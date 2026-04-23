/**
 * @file test_servo_t75.cpp
 * @brief Unit tests for SRT Servo T75 HV driver math layer.
 *
 * Covers the platform-independent functions:
 *   - angle_to_pulse_us:  angle (deg) → pulse width (µs) with trim,
 *                         direction sign, software clamp, hardware clamp.
 *   - slew_step:          rate-limited move from current pulse toward target.
 *
 * No hardware dependency — exercised on host via Google Test.
 */

#include <cstdint>
#include <gtest/gtest.h>

#include "drivers/servo_t75_math.h"

using acs::servo_t75::Limits;

namespace
{

constexpr Limits kDefault = {
    .neutral_us     = 1500,
    .min_us         = 1300,
    .max_us         = 1700,
    .direction_sign = +1,
    .us_per_deg     = 11.111f,
};

}  // namespace

/* ═════════════════════════════════════════════════════════════════════
 * angle_to_pulse_us
 * ═════════════════════════════════════════════════════════════════════ */

TEST(ServoAngleToPulse, NeutralIsZeroDeg)
{
    EXPECT_EQ(acs::servo_t75::angle_to_pulse_us(0.0f, kDefault, 15.0f), 1500);
}

TEST(ServoAngleToPulse, PositiveDirectionPushesAbove1500)
{
    /* +10° × 11.111 ≈ 111 µs → 1611 µs. */
    const auto pulse = acs::servo_t75::angle_to_pulse_us(10.0f, kDefault, 15.0f);
    EXPECT_GE(pulse, 1610);
    EXPECT_LE(pulse, 1612);
}

TEST(ServoAngleToPulse, InvertedDirectionPushesBelow1500)
{
    Limits inv          = kDefault;
    inv.direction_sign  = -1;
    const auto pulse    = acs::servo_t75::angle_to_pulse_us(10.0f, inv, 15.0f);
    EXPECT_GE(pulse, 1388);
    EXPECT_LE(pulse, 1390);
}

TEST(ServoAngleToPulse, NegativeAngleSymmetric)
{
    const auto pos = acs::servo_t75::angle_to_pulse_us(+5.0f, kDefault, 15.0f);
    const auto neg = acs::servo_t75::angle_to_pulse_us(-5.0f, kDefault, 15.0f);
    /* +5° and -5° should be symmetric around 1500 (both directions equal). */
    EXPECT_EQ(pos - 1500, 1500 - neg);
}

TEST(ServoAngleToPulse, TrimOffsetShiftsNeutral)
{
    Limits trimmed       = kDefault;
    trimmed.neutral_us   = 1512;
    EXPECT_EQ(acs::servo_t75::angle_to_pulse_us(0.0f, trimmed, 15.0f), 1512);
}

TEST(ServoAngleToPulse, SoftwareDeflectionClampApplied)
{
    /* max_angle_deg=15: commanding 25° should produce the same value as 15°. */
    const auto p_25 = acs::servo_t75::angle_to_pulse_us(25.0f, kDefault, 15.0f);
    const auto p_15 = acs::servo_t75::angle_to_pulse_us(15.0f, kDefault, 15.0f);
    EXPECT_EQ(p_25, p_15);
}

TEST(ServoAngleToPulse, HardwareClampMin)
{
    /* Loose software cap of 90° but tight hardware min of 1300 µs. */
    const auto pulse = acs::servo_t75::angle_to_pulse_us(-90.0f, kDefault, 90.0f);
    EXPECT_EQ(pulse, 1300);
}

TEST(ServoAngleToPulse, HardwareClampMax)
{
    const auto pulse = acs::servo_t75::angle_to_pulse_us(+90.0f, kDefault, 90.0f);
    EXPECT_EQ(pulse, 1700);
}

TEST(ServoAngleToPulse, NegativeMaxAngleHandled)
{
    /* max_angle_deg given as -15 should be treated as +15. */
    const auto pos = acs::servo_t75::angle_to_pulse_us(20.0f, kDefault, -15.0f);
    const auto ref = acs::servo_t75::angle_to_pulse_us(15.0f, kDefault, 15.0f);
    EXPECT_EQ(pos, ref);
}

/* ═════════════════════════════════════════════════════════════════════
 * slew_step
 * ═════════════════════════════════════════════════════════════════════ */

TEST(ServoSlew, NoMovementWhenAtTarget)
{
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1500, 2.0f, 10), 1500);
}

TEST(ServoSlew, StepsTowardTargetUpward)
{
    /* 2 µs/ms * 10 ms = 20 µs step → 1500 + 20 = 1520. */
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1700, 2.0f, 10), 1520);
}

TEST(ServoSlew, StepsTowardTargetDownward)
{
    EXPECT_EQ(acs::servo_t75::slew_step(1700, 1500, 2.0f, 10), 1680);
}

TEST(ServoSlew, ReachesTargetCumulatively)
{
    uint16_t cur = 1500;
    for (int i = 0; i < 10; ++i)
    {
        cur = acs::servo_t75::slew_step(cur, 1700, 2.0f, 10);
    }
    /* 10 steps × 20 µs = 200 µs → exactly 1700. */
    EXPECT_EQ(cur, 1700);
}

TEST(ServoSlew, DoesNotOvershoot)
{
    /* Single step capable of moving 200 µs but target is only 50 µs away. */
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1550, 100.0f, 10), 1550);
}

TEST(ServoSlew, ZeroDtKeepsCurrent)
{
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1700, 2.0f, 0), 1500);
}

TEST(ServoSlew, ZeroRateKeepsCurrent)
{
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1700, 0.0f, 10), 1500);
}

TEST(ServoSlew, NegativeRateKeepsCurrent)
{
    EXPECT_EQ(acs::servo_t75::slew_step(1500, 1700, -1.0f, 10), 1500);
}
