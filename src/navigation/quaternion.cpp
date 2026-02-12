/**
 * @file quaternion.cpp
 * @brief Quaternion operations — Eigen-backed implementation.
 */

#include "navigation/quaternion.h"

#include <algorithm>
#include <cmath>

namespace acs {
namespace nav {

/* ── Construction ────────────────────────────────────────────────────────── */

Quat quat_from_rotation_vector(const Vec3& rv)
{
    const float angle = rv.norm();
    if (angle < 1e-10f) {
        /* Small angle: first-order Taylor of exp(rv/2).
         * q ≈ [1, rv/2]  (then normalize to stay on S³). */
        return Quat(1.0f, rv.x() * 0.5f, rv.y() * 0.5f, rv.z() * 0.5f)
            .normalized();
    }
    const float half_angle = angle * 0.5f;
    const float s = std::sin(half_angle) / angle;   /* sin(θ/2) / θ */
    return Quat(std::cos(half_angle), rv.x() * s, rv.y() * s, rv.z() * s);
}

Quat quat_from_axis_angle(const Vec3& axis, float angle_rad)
{
    const float n = axis.norm();
    if (n < 1e-10f) {
        return Quat::Identity();
    }
    const Vec3 u = axis / n;
    const float half = angle_rad * 0.5f;
    const float s = std::sin(half);
    return Quat(std::cos(half), u.x() * s, u.y() * s, u.z() * s);
}

Quat quat_from_euler(float roll, float pitch, float yaw)
{
    /* ZYX: R = Rz(ψ) · Ry(θ) · Rx(φ)  →  q = qz ⊗ qy ⊗ qx */
    const Quat qx(Eigen::AngleAxisf(roll,  Vec3::UnitX()));
    const Quat qy(Eigen::AngleAxisf(pitch, Vec3::UnitY()));
    const Quat qz(Eigen::AngleAxisf(yaw,   Vec3::UnitZ()));
    return (qz * qy * qx).normalized();
}

/* ── Core operations ─────────────────────────────────────────────────────── */

Quat quat_normalize(const Quat& q)
{
    const float n = q.norm();
    if (n < 1e-10f || std::isnan(n)) {
        return Quat::Identity();
    }
    return Quat(q.coeffs() / n);
}

/* ── Euler extraction ────────────────────────────────────────────────────── */

void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw)
{
    /*
     * From DCM (ZYX convention):
     *   roll  = atan2(R(2,1), R(2,2))
     *   pitch = -asin(R(2,0))
     *   yaw   = atan2(R(1,0), R(0,0))
     */
    const Mat3 R = q.toRotationMatrix();

    const float sinp = -R(2, 0);
    /* Clamp for numerical safety near ±90° pitch (gimbal lock region). */
    const float sinp_clamped = std::clamp(sinp, -1.0f, 1.0f);

    roll  = std::atan2(R(2, 1), R(2, 2));
    pitch = std::asin(sinp_clamped);
    yaw   = std::atan2(R(1, 0), R(0, 0));
}

/* ── Integration ─────────────────────────────────────────────────────────── */

Quat quat_integrate(const Quat& q, const Vec3& omega, float dt)
{
    /* q_new = q ⊗ quat_from_rotation_vector(ω · dt) */
    const Quat dq = quat_from_rotation_vector(omega * dt);
    return (q * dq).normalized();
}

/* ── Error metrics ───────────────────────────────────────────────────────── */

float quat_error_angle(const Quat& a, const Quat& b)
{
    /* θ = 2 · acos(|a · b|)
     * Dot product of quaternion coefficients. */
    float dot = std::abs(a.w() * b.w() + a.x() * b.x() +
                         a.y() * b.y() + a.z() * b.z());
    dot = std::min(1.0f, dot);   /* clamp for numerical safety */
    return 2.0f * std::acos(dot);
}

Vec3 quat_error_vector(const Quat& current, const Quat& desired)
{
    Quat q_err = current.conjugate() * desired;

    /* Ensure shortest rotation path (q and -q represent the same rotation). */
    if (q_err.w() < 0.0f) {
        q_err.coeffs() = -q_err.coeffs();
    }

    /* Small-angle approximation: error ≈ 2 · [qx, qy, qz]. */
    return 2.0f * q_err.vec();
}

}  // namespace nav
}  // namespace acs
