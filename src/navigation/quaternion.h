/**
 * @file quaternion.h
 * @brief Quaternion algebra for attitude representation (Eigen-backed).
 *
 * Conventions:
 *   - Hamilton quaternion: q = w + xi + yj + zk
 *   - Rotation semantics: body-frame -> NED (world) frame
 *   - Euler angles: ZYX aerospace convention (yaw-pitch-roll)
 *   - All angles in radians, all units SI
 *
 * Thread safety: all functions are pure (no shared mutable state).
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace acs {
namespace nav {

/* ── Type aliases ────────────────────────────────────────────────────────── */
using Quat = Eigen::Quaternionf;
using Vec3 = Eigen::Vector3f;
using Mat3 = Eigen::Matrix3f;

/* ── Construction ────────────────────────────────────────────────────────── */

/** @brief Identity quaternion (no rotation). */
inline Quat quat_identity() { return Quat::Identity(); }

/**
 * @brief Create quaternion from rotation vector (exponential map).
 * @param rv  Rotation vector [rad]. Direction = axis, magnitude = angle.
 * @return Unit quaternion representing the rotation.
 */
Quat quat_from_rotation_vector(const Vec3& rv);

/**
 * @brief Create quaternion from axis-angle representation.
 * @param axis      Rotation axis (will be normalized internally).
 * @param angle_rad Rotation angle [rad].
 */
Quat quat_from_axis_angle(const Vec3& axis, float angle_rad);

/**
 * @brief Create quaternion from ZYX Euler angles.
 * @param roll  φ — rotation about X (forward)  [rad]
 * @param pitch θ — rotation about Y (right)    [rad]
 * @param yaw   ψ — rotation about Z (down)     [rad]
 */
Quat quat_from_euler(float roll, float pitch, float yaw);

/* ── Core operations ─────────────────────────────────────────────────────── */

/** @brief Normalize quaternion to unit length. Returns identity on NaN. */
Quat quat_normalize(const Quat& q);

/** @brief Quaternion conjugate (= inverse for unit quaternions). */
inline Quat quat_conjugate(const Quat& q) { return q.conjugate(); }

/** @brief Hamilton product: result = a * b. */
inline Quat quat_multiply(const Quat& a, const Quat& b) { return a * b; }

/**
 * @brief Rotate vector from body frame to NED frame.
 * @param q  Body-to-NED quaternion.
 * @param v  Vector in body frame.
 * @return   Vector in NED frame.
 */
inline Vec3 quat_rotate_vector(const Quat& q, const Vec3& v) { return q * v; }

/**
 * @brief Convert quaternion to Direction Cosine Matrix.
 * @param q  Unit quaternion (body -> NED).
 * @return   3×3 rotation matrix.
 */
inline Mat3 quat_to_dcm(const Quat& q) { return q.toRotationMatrix(); }

/* ── Euler extraction (logging only!) ────────────────────────────────────── */

/**
 * @brief Extract ZYX Euler angles from quaternion.
 * @note  For logging/display ONLY — never use Euler in flight computations.
 * @param q             Unit quaternion.
 * @param[out] roll     φ [rad]
 * @param[out] pitch    θ [rad]
 * @param[out] yaw      ψ [rad]
 */
void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw);

/* ── Integration (for IMU / ESKF predict) ────────────────────────────────── */

/**
 * @brief Integrate quaternion with angular velocity (1st-order).
 *
 * Computes  q_new = q ⊗ exp(ω · dt)
 * where exp() is the rotation-vector exponential map.
 *
 * @param q     Current orientation quaternion.
 * @param omega Angular velocity in body frame [rad/s].
 * @param dt    Time step [s].
 * @return      Updated, normalized quaternion.
 */
Quat quat_integrate(const Quat& q, const Vec3& omega, float dt);

/* ── Error metrics (for attitude control) ────────────────────────────────── */

/**
 * @brief Geodesic angle between two orientations.
 * @return Angle in [0, π] [rad].
 */
float quat_error_angle(const Quat& a, const Quat& b);

/**
 * @brief Rotation error vector from current to desired orientation.
 *
 * Computes  q_err = conj(current) ⊗ desired,
 * ensures shortest path, returns 2·[qx, qy, qz] (small-angle approx).
 *
 * @param current  Current orientation.
 * @param desired  Desired orientation.
 * @return Error rotation vector in body frame [rad].
 */
Vec3 quat_error_vector(const Quat& current, const Quat& desired);

}  // namespace nav
}  // namespace acs
