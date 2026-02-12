/**
 * @file test_quaternion.cpp
 * @brief Unit tests for acs::nav quaternion library.
 *
 * Coverage:
 *   - normalize, multiply, conjugate
 *   - from_rotation_vector, from_axis_angle, from_euler
 *   - rotate_vector, to_dcm, to_euler
 *   - integrate (gyro)
 *   - error_angle, error_vector
 *   - edge cases: identity, zero rotation, 180° flip, gimbal lock
 *   - numerical stability (repeated normalize)
 */

#include <gtest/gtest.h>

#include "navigation/quaternion.h"

#include <cmath>

using namespace acs::nav;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
static constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);
static constexpr float TOL     = 1e-5f;
static constexpr float TOL_DEG = 0.1f * DEG2RAD;   /* 0.1° in rad */

/** Check that quaternion is approximately equal (handles q ≈ -q ambiguity). */
static void EXPECT_QUAT_NEAR(const Quat& a, const Quat& b, float tol = TOL)
{
    /* q and -q represent the same rotation */
    float dot = std::fabs(a.w() * b.w() + a.x() * b.x() +
                          a.y() * b.y() + a.z() * b.z());
    EXPECT_NEAR(dot, 1.0f, tol)
        << "Quaternions differ: ["
        << a.w() << "," << a.x() << "," << a.y() << "," << a.z() << "] vs ["
        << b.w() << "," << b.x() << "," << b.y() << "," << b.z() << "]";
}

static void EXPECT_VEC3_NEAR(const Vec3& a, const Vec3& b, float tol = TOL)
{
    EXPECT_NEAR(a.x(), b.x(), tol);
    EXPECT_NEAR(a.y(), b.y(), tol);
    EXPECT_NEAR(a.z(), b.z(), tol);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Normalize
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatNormalize, UnnormalizedBecomesUnit)
{
    Quat q(1.0f, 1.0f, 1.0f, 1.0f);
    Quat n = quat_normalize(q);
    EXPECT_NEAR(n.norm(), 1.0f, TOL);
}

TEST(QuatNormalize, AlreadyUnit)
{
    Quat q = quat_identity();
    Quat n = quat_normalize(q);
    EXPECT_NEAR(n.norm(), 1.0f, TOL);
    EXPECT_NEAR(n.w(), 1.0f, TOL);
}

TEST(QuatNormalize, ZeroReturnsIdentity)
{
    Quat q(0.0f, 0.0f, 0.0f, 0.0f);
    Quat n = quat_normalize(q);
    EXPECT_NEAR(n.w(), 1.0f, TOL);
    EXPECT_NEAR(n.norm(), 1.0f, TOL);
}

TEST(QuatNormalize, NaNReturnsIdentity)
{
    Quat q(NAN, 0.0f, 0.0f, 0.0f);
    Quat n = quat_normalize(q);
    EXPECT_NEAR(n.w(), 1.0f, TOL);
}

TEST(QuatNormalize, RepeatedNormalize1000x)
{
    /* Numerical stability: 1000× normalize must not drift. */
    Quat q = quat_from_euler(0.3f, 0.5f, 1.2f);
    for (int i = 0; i < 1000; ++i) {
        q = quat_normalize(q);
    }
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Multiply & Conjugate
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatMultiply, QTimesConjugateIsIdentity)
{
    Quat q = quat_from_euler(0.1f, 0.2f, 0.3f);
    Quat result = quat_multiply(q, quat_conjugate(q));
    EXPECT_QUAT_NEAR(result, quat_identity());
}

TEST(QuatMultiply, IdentityIsNeutral)
{
    Quat q = quat_from_euler(0.5f, -0.3f, 1.0f);
    EXPECT_QUAT_NEAR(quat_multiply(quat_identity(), q), q);
    EXPECT_QUAT_NEAR(quat_multiply(q, quat_identity()), q);
}

TEST(QuatMultiply, TwoRotationsCompose)
{
    /* 90° about Z then 90° about Z = 180° about Z */
    Quat q90z = quat_from_axis_angle(Vec3::UnitZ(), 90.0f * DEG2RAD);
    Quat q180z = quat_multiply(q90z, q90z);
    Quat expected = quat_from_axis_angle(Vec3::UnitZ(), 180.0f * DEG2RAD);
    EXPECT_QUAT_NEAR(q180z, expected);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Construction: rotation vector
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatFromRotVec, ZeroIsIdentity)
{
    Quat q = quat_from_rotation_vector(Vec3::Zero());
    EXPECT_QUAT_NEAR(q, quat_identity());
}

TEST(QuatFromRotVec, NinetyDegAboutZ)
{
    float angle = 90.0f * DEG2RAD;
    Vec3 rv(0.0f, 0.0f, angle);
    Quat q = quat_from_rotation_vector(rv);
    Quat expected = quat_from_axis_angle(Vec3::UnitZ(), angle);
    EXPECT_QUAT_NEAR(q, expected);
}

TEST(QuatFromRotVec, SmallAngle)
{
    /* Very small rotation — tests Taylor branch */
    Vec3 rv(1e-12f, 0.0f, 0.0f);
    Quat q = quat_from_rotation_vector(rv);
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
    EXPECT_NEAR(q.w(), 1.0f, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Construction: axis-angle
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatFromAxisAngle, ZeroAngleIsIdentity)
{
    Quat q = quat_from_axis_angle(Vec3::UnitX(), 0.0f);
    EXPECT_QUAT_NEAR(q, quat_identity());
}

TEST(QuatFromAxisAngle, ZeroAxisIsIdentity)
{
    Quat q = quat_from_axis_angle(Vec3::Zero(), 1.0f);
    EXPECT_QUAT_NEAR(q, quat_identity());
}

TEST(QuatFromAxisAngle, OneEightyDeg)
{
    Quat q = quat_from_axis_angle(Vec3::UnitX(), static_cast<float>(M_PI));
    /* w ≈ 0, x ≈ 1 */
    EXPECT_NEAR(std::fabs(q.w()), 0.0f, TOL);
    EXPECT_NEAR(std::fabs(q.x()), 1.0f, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Construction: Euler
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatFromEuler, ZerosIsIdentity)
{
    Quat q = quat_from_euler(0.0f, 0.0f, 0.0f);
    EXPECT_QUAT_NEAR(q, quat_identity());
}

TEST(QuatFromEuler, PureYaw90)
{
    Quat q = quat_from_euler(0.0f, 0.0f, 90.0f * DEG2RAD);
    Quat expected = quat_from_axis_angle(Vec3::UnitZ(), 90.0f * DEG2RAD);
    EXPECT_QUAT_NEAR(q, expected);
}

TEST(QuatFromEuler, PurePitch45)
{
    Quat q = quat_from_euler(0.0f, 45.0f * DEG2RAD, 0.0f);
    Quat expected = quat_from_axis_angle(Vec3::UnitY(), 45.0f * DEG2RAD);
    EXPECT_QUAT_NEAR(q, expected);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Rotate vector
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatRotateVector, NinetyDegAboutZ)
{
    /* Roadmap test: 90° about Z rotates {1,0,0} → {0,1,0} */
    Quat q = quat_from_axis_angle(Vec3::UnitZ(), 90.0f * DEG2RAD);
    Vec3 result = quat_rotate_vector(q, Vec3::UnitX());
    EXPECT_VEC3_NEAR(result, Vec3::UnitY());
}

TEST(QuatRotateVector, IdentityLeavesVectorUnchanged)
{
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 result = quat_rotate_vector(quat_identity(), v);
    EXPECT_VEC3_NEAR(result, v);
}

TEST(QuatRotateVector, OneEightyAboutX)
{
    Quat q = quat_from_axis_angle(Vec3::UnitX(), static_cast<float>(M_PI));
    Vec3 result = quat_rotate_vector(q, Vec3::UnitY());
    EXPECT_VEC3_NEAR(result, Vec3(0.0f, -1.0f, 0.0f));
}

TEST(QuatRotateVector, ConjugateIsInverse)
{
    Quat q = quat_from_euler(0.3f, 0.5f, 1.2f);
    Vec3 v(1.0f, 2.0f, 3.0f);
    Vec3 rotated   = quat_rotate_vector(q, v);
    Vec3 unrotated = quat_rotate_vector(quat_conjugate(q), rotated);
    EXPECT_VEC3_NEAR(unrotated, v);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DCM
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatToDcm, IdentityIsDiag)
{
    Mat3 R = quat_to_dcm(quat_identity());
    EXPECT_NEAR(R(0, 0), 1.0f, TOL);
    EXPECT_NEAR(R(1, 1), 1.0f, TOL);
    EXPECT_NEAR(R(2, 2), 1.0f, TOL);
    EXPECT_NEAR(R(0, 1), 0.0f, TOL);
}

TEST(QuatToDcm, ConsistentWithRotateVector)
{
    Quat q = quat_from_euler(0.3f, 0.5f, 1.2f);
    Vec3 v(1.0f, -0.5f, 2.0f);
    Vec3 via_quat = quat_rotate_vector(q, v);
    Vec3 via_dcm  = quat_to_dcm(q) * v;
    EXPECT_VEC3_NEAR(via_quat, via_dcm);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Euler extraction
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatToEuler, IdentityIsZero)
{
    float r, p, y;
    quat_to_euler(quat_identity(), r, p, y);
    EXPECT_NEAR(r, 0.0f, TOL);
    EXPECT_NEAR(p, 0.0f, TOL);
    EXPECT_NEAR(y, 0.0f, TOL);
}

TEST(QuatToEuler, RoundTrip)
{
    float r_in = 10.0f * DEG2RAD;
    float p_in = 25.0f * DEG2RAD;
    float y_in = -60.0f * DEG2RAD;

    Quat q = quat_from_euler(r_in, p_in, y_in);
    float r_out, p_out, y_out;
    quat_to_euler(q, r_out, p_out, y_out);

    EXPECT_NEAR(r_out, r_in, TOL_DEG);
    EXPECT_NEAR(p_out, p_in, TOL_DEG);
    EXPECT_NEAR(y_out, y_in, TOL_DEG);
}

TEST(QuatToEuler, GimbalLock90Pitch)
{
    /* pitch = +90° (gimbal lock) — roll and yaw degenerate but sum is defined.
     * Just verify no NaN/crash. */
    Quat q = quat_from_euler(0.0f, 90.0f * DEG2RAD, 0.0f);
    float r, p, y;
    quat_to_euler(q, r, p, y);
    EXPECT_FALSE(std::isnan(r));
    EXPECT_FALSE(std::isnan(p));
    EXPECT_FALSE(std::isnan(y));
    EXPECT_NEAR(p, 90.0f * DEG2RAD, TOL_DEG);
}

TEST(QuatToEuler, NegativePitch)
{
    float r_in = 0.0f;
    float p_in = -30.0f * DEG2RAD;
    float y_in = 0.0f;

    Quat q = quat_from_euler(r_in, p_in, y_in);
    float r_out, p_out, y_out;
    quat_to_euler(q, r_out, p_out, y_out);

    EXPECT_NEAR(p_out, p_in, TOL_DEG);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Integration (gyroscope)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatIntegrate, ZeroOmegaNoChange)
{
    Quat q = quat_from_euler(0.1f, 0.2f, 0.3f);
    Quat q2 = quat_integrate(q, Vec3::Zero(), 0.001f);
    EXPECT_QUAT_NEAR(q, q2);
}

TEST(QuatIntegrate, SmallYawRotation)
{
    /* Roadmap test: identity + omega_z=1 rad/s, dt=0.001 → small yaw */
    Quat q = quat_identity();
    Vec3 omega(0.0f, 0.0f, 1.0f);  /* 1 rad/s about Z */
    Quat q2 = quat_integrate(q, omega, 0.001f);

    float r, p, y;
    quat_to_euler(q2, r, p, y);

    EXPECT_NEAR(r, 0.0f, TOL_DEG);
    EXPECT_NEAR(p, 0.0f, TOL_DEG);
    EXPECT_NEAR(y, 0.001f, 1e-4f);  /* expect ~0.001 rad yaw */
}

TEST(QuatIntegrate, ConstantRotation90Deg)
{
    /* Constant 90°/s about Z for 1 second (1000 steps × 1 ms).
     * Final yaw ≈ 90° = π/2 rad. */
    Quat q = quat_identity();
    const float omega_z = 90.0f * DEG2RAD;   /* 90°/s  */
    const Vec3 omega(0.0f, 0.0f, omega_z);
    const float dt = 0.001f;

    for (int i = 0; i < 1000; ++i) {
        q = quat_integrate(q, omega, dt);
    }

    float r, p, y;
    quat_to_euler(q, r, p, y);

    EXPECT_NEAR(y, 90.0f * DEG2RAD, 1.0f * DEG2RAD);  /* ±1° tolerance */
}

TEST(QuatIntegrate, ResultIsNormalized)
{
    Quat q = quat_identity();
    Vec3 omega(1.0f, 0.5f, -0.3f);
    for (int i = 0; i < 10000; ++i) {
        q = quat_integrate(q, omega, 0.001f);
    }
    EXPECT_NEAR(q.norm(), 1.0f, TOL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Error angle
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatErrorAngle, SameQuatIsZero)
{
    Quat q = quat_from_euler(0.3f, 0.5f, 1.2f);
    EXPECT_NEAR(quat_error_angle(q, q), 0.0f, TOL);
}

TEST(QuatErrorAngle, NegativeQuatIsZero)
{
    /* q and -q are the same rotation */
    Quat q = quat_from_euler(0.3f, 0.5f, 1.2f);
    Quat neg_q(-(q.w()), -(q.x()), -(q.y()), -(q.z()));
    EXPECT_NEAR(quat_error_angle(q, neg_q), 0.0f, TOL);
}

TEST(QuatErrorAngle, NinetyDeg)
{
    Quat a = quat_identity();
    Quat b = quat_from_axis_angle(Vec3::UnitX(), 90.0f * DEG2RAD);
    EXPECT_NEAR(quat_error_angle(a, b), 90.0f * DEG2RAD, TOL_DEG);
}

TEST(QuatErrorAngle, OneEightyDeg)
{
    Quat a = quat_identity();
    Quat b = quat_from_axis_angle(Vec3::UnitZ(),
                                   static_cast<float>(M_PI));
    EXPECT_NEAR(quat_error_angle(a, b), static_cast<float>(M_PI), TOL_DEG);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Error vector (attitude control)
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatErrorVector, SameQuatIsZero)
{
    Quat q = quat_from_euler(0.1f, 0.2f, 0.3f);
    Vec3 err = quat_error_vector(q, q);
    EXPECT_VEC3_NEAR(err, Vec3::Zero());
}

TEST(QuatErrorVector, SmallPitchError)
{
    Quat current = quat_identity();
    Quat desired = quat_from_euler(0.0f, 10.0f * DEG2RAD, 0.0f);
    Vec3 err = quat_error_vector(current, desired);

    /* Error should be approximately [0, 0.1745, 0] (10° in rad about Y) */
    EXPECT_NEAR(err.x(), 0.0f, TOL);
    EXPECT_NEAR(err.y(), 10.0f * DEG2RAD, 1.0f * DEG2RAD);
    EXPECT_NEAR(err.z(), 0.0f, TOL);
}

TEST(QuatErrorVector, DirectionIsCorrect)
{
    /* current = 0°, desired = +5° roll  →  positive X error (roll right) */
    Quat current = quat_identity();
    Quat desired = quat_from_euler(5.0f * DEG2RAD, 0.0f, 0.0f);
    Vec3 err = quat_error_vector(current, desired);
    EXPECT_GT(err.x(), 0.0f);
}

TEST(QuatErrorVector, ShortestPath)
{
    /* Ensure we always pick the shortest path (< 180°), even when
     * q_err.w < 0. The small-angle approximation 2·vec(q) saturates
     * near 2.0 rad for large rotations — that's expected and fine
     * for attitude control (correct direction, bounded magnitude). */
    Quat current = quat_identity();
    Quat desired = quat_from_axis_angle(Vec3::UnitZ(), 170.0f * DEG2RAD);
    Vec3 err = quat_error_vector(current, desired);

    /* Direction must be correct (positive Z for positive yaw error). */
    EXPECT_GT(err.z(), 0.0f);

    /* Magnitude is large (>1 rad) but bounded by 2.0 due to approx. */
    EXPECT_GT(err.norm(), 1.0f);
    EXPECT_LE(err.norm(), 2.0f + TOL);

    /* Verify it's still shorter than going the "long way" (190°).
     * 190° would mean negative Z direction if shortest path failed. */
    Quat desired2 = quat_from_axis_angle(Vec3::UnitZ(), -190.0f * DEG2RAD);
    Vec3 err2 = quat_error_vector(current, desired2);
    EXPECT_GT(err2.z(), 0.0f);  /* should still be +Z (170° short way) */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Cross-function consistency
 * ═══════════════════════════════════════════════════════════════════════════ */

TEST(QuatConsistency, RotVecAndAxisAngleAgree)
{
    float angle = 37.0f * DEG2RAD;
    Vec3 axis = Vec3(1.0f, 2.0f, 3.0f).normalized();
    Vec3 rv = axis * angle;

    Quat from_rv = quat_from_rotation_vector(rv);
    Quat from_aa = quat_from_axis_angle(axis, angle);
    EXPECT_QUAT_NEAR(from_rv, from_aa);
}

TEST(QuatConsistency, FromEulerToEulerRoundTripMultiple)
{
    /* Test several angle combinations */
    float angles[][3] = {
        {0.0f, 0.0f, 0.0f},
        {30.0f, 0.0f, 0.0f},
        {0.0f, 45.0f, 0.0f},
        {0.0f, 0.0f, -120.0f},
        {15.0f, -25.0f, 60.0f},
        {-10.0f, 80.0f, -170.0f},
    };

    for (auto& a : angles) {
        float r_in = a[0] * DEG2RAD;
        float p_in = a[1] * DEG2RAD;
        float y_in = a[2] * DEG2RAD;

        Quat q = quat_from_euler(r_in, p_in, y_in);
        float r_out, p_out, y_out;
        quat_to_euler(q, r_out, p_out, y_out);

        EXPECT_NEAR(r_out, r_in, TOL_DEG)
            << "roll fail for [" << a[0] << "," << a[1] << "," << a[2] << "]";
        EXPECT_NEAR(p_out, p_in, TOL_DEG)
            << "pitch fail for [" << a[0] << "," << a[1] << "," << a[2] << "]";
        EXPECT_NEAR(y_out, y_in, TOL_DEG)
            << "yaw fail for [" << a[0] << "," << a[1] << "," << a[2] << "]";
    }
}
