#pragma once
//
// dh_chain.hpp
// Minimal Denavit-Hartenberg description for a 4-DOF serial arm.
// Convention: standard DH (Craig), transform = Rot_z(theta) * Trans_z(d)
//             * Trans_x(a) * Rot_x(alpha).
//

#include <array>
#include <cstddef>

namespace kin {

constexpr std::size_t kDof = 4;

// One row of the DH table. Units: metres and radians.
struct DhParams {
    double theta_offset;  // fixed rotation about previous z (rad)
    double d;             // link offset along previous z (m)
    double a;             // link length along common normal (m)
    double alpha;         // twist about common normal (rad)
    double q_min;         // lower joint limit (rad)
    double q_max;         // upper joint limit (rad)
};

// Row-major 4x4 homogeneous transform.
struct Transform4 {
    std::array<double, 16> m{};

    static Transform4 identity();
    Transform4 operator*(const Transform4& rhs) const;
    // Rotation block applied to a 3-vector (upper-left 3x3).
    std::array<double, 3> rotate(const std::array<double, 3>& v) const;
};

// Joint configuration, radians.
struct JointConfig {
    std::array<double, kDof> q{};
    double operator[](std::size_t i) const { return q[i]; }
    double& operator[](std::size_t i) { return q[i]; }
};

// Cartesian pose in world frame.
struct Pose {
    std::array<double, 3> p{};       // position (m)
    Transform4 rot;                  // orientation only
};

}  // namespace kin
