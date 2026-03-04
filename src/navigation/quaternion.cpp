/**
 * @file quaternion.cpp
 * @brief Quaternion operations — Eigen-backed implementation.
 */

#include "navigation/quaternion.h"

#include <algorithm>
#include <cmath>

namespace acs::nav {

/* ── Construction ────────────────────────────────────────────────────────── */

Quat quat_from_rotation_vector(const Vec3 &rv)
{
    const float angle = rv.norm();
    if (angle < 1e-10f)
    {
        /* Small angle: first-order Taylor of exp(rv/2).
         * q ≈ [1, rv/2]  (then normalize to stay on S³). */
        return Quat(1.0f, rv.x() * 0.5f, rv.y() * 0.5f, rv.z() * 0.5f)
            .normalized();
    }
    const float half_angle = angle * 0.5f;
    const float s          = std::sin(half_angle) / angle; /* sin(θ/2) / θ */
    return Quat(std::cos(half_angle), rv.x() * s, rv.y() * s, rv.z() * s);
}

Quat quat_from_axis_angle(const Vec3 &axis, float angle_rad)
{
    const float n = axis.norm();
    if (n < 1e-10f)
    {
        return Quat::Identity();
    }
    const Vec3  u    = axis / n;
    const float half = angle_rad * 0.5f;
    const float s    = std::sin(half);
    return Quat(std::cos(half), u.x() * s, u.y() * s, u.z() * s);
}

Quat quat_from_euler(float roll, float pitch, float yaw)
{
    /* ZYX: R = Rz(ψ) · Ry(θ) · Rx(φ)  →  q = qz ⊗ qy ⊗ qx
     * Direct formula avoids constructing 3 intermediate quaternions. */
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);

    return Quat(cr * cp * cy + sr * sp * sy,   /* w */
                sr * cp * cy - cr * sp * sy,    /* x */
                cr * sp * cy + sr * cp * sy,    /* y */
                cr * cp * sy - sr * sp * cy);   /* z */
}

/* ── Core operations ─────────────────────────────────────────────────────── */

Quat quat_normalize(const Quat &q)
{
    const float n = q.norm();
    if (n < 1e-10f || std::isnan(n))
    {
        return Quat::Identity();
    }
    return Quat(q.coeffs() / n);
}

/* ── Euler extraction ────────────────────────────────────────────────────── */

void quat_to_euler(const Quat &q, float &roll, float &pitch,
                   float &yaw)
{
    /*
     * Direct extraction from quaternion components (ZYX convention).
     * Avoids computing the full 3×3 DCM — only the 5 needed elements.
     *
     *   R(2,0) = 2(qx·qz - qw·qy)
     *   R(2,1) = 2(qy·qz + qw·qx)
     *   R(2,2) = qw² - qx² - qy² + qz²
     *   R(1,0) = 2(qx·qy + qw·qz)
     *   R(0,0) = qw² + qx² - qy² - qz²
     */
    const float w = q.w();
    const float x = q.x();
    const float y = q.y();
    const float z = q.z();

    /* sinp = -R(2,0) = 2(qw·qy - qx·qz) */
    const float sinp = 2.0f * (w * y - x * z);
    const float sinp_clamped = std::clamp(sinp, -1.0f, 1.0f);

    roll  = std::atan2(2.0f * (y * z + w * x),
                       w * w - x * x - y * y + z * z);
    pitch = std::asin(sinp_clamped);
    yaw   = std::atan2(2.0f * (x * y + w * z),
                       w * w + x * x - y * y - z * z);
}

/* ── Integration ─────────────────────────────────────────────────────────── */

Quat quat_integrate(const Quat &q, const Vec3 &omega, float dt)
{
    /* q_new = q ⊗ quat_from_rotation_vector(ω · dt) */
    const Quat dq = quat_from_rotation_vector(omega * dt);
    return (q * dq).normalized();
}

/* ── Error metrics ───────────────────────────────────────────────────────── */

float quat_error_angle(const Quat &a, const Quat &b)
{
    /* θ = 2 · acos(|a · b|) */
    float dot = std::abs(a.coeffs().dot(b.coeffs()));
    dot = std::min(1.0f, dot); /* clamp for numerical safety */
    return 2.0f * std::acos(dot);
}

Vec3 quat_error_vector(const Quat &current, const Quat &desired)
{
    Quat q_err = current.conjugate() * desired;

    /* Ensure shortest rotation path (q and -q represent the same rotation). */
    if (q_err.w() < 0.0f)
    {
        q_err.coeffs() = -q_err.coeffs();
    }

    /* Small-angle approximation: error ≈ 2 · [qx, qy, qz]. */
    return 2.0f * q_err.vec();
}

}  // namespace acs::nav
