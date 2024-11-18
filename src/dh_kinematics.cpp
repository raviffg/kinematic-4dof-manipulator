//
// dh_kinematics.cpp
// Forward kinematics + damped-least-squares (DLS) inverse kinematics for a
// 4-DOF serial manipulator. Bench arm: 4x NEMA-17, 1:50 planetary on the
// shoulder and elbow, direct drive on the wrist roll.
//
// The DLS step is the Levenberg-Marquardt flavoured update:
//
//     dq = J^T (J J^T + lambda^2 I)^-1 * e
//
// which stays finite near the elbow singularity where the plain pseudo-inverse
// blows up. lambda is raised when the step size stalls and relaxed again once
// progress resumes, so damping only kicks in when we actually need it.
//
#include "dh_chain.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace kin {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Bench-measured DH table, millimetres converted to metres.
// Numbers came off the caliper pass on 2026-03-14, do not "tidy" them.
const std::array<DhParams, kDof> kTable = {{
    // theta_off   d        a        alpha      q_min     q_max
    {  0.0,     0.1800,  0.0000,   kPi / 2,  -2.967,   2.967 },  // base yaw
    {  0.0,     0.0000,  0.2400,   0.000,    -1.745,   1.745 },  // shoulder
    {  0.0,     0.0000,  0.1950,   0.000,    -2.618,   0.087 },  // elbow
    {  0.0,     0.0750,  0.0000,   0.000,    -3.141,   3.141 },  // wrist roll
}};

constexpr int kMaxIterations = 250;
constexpr double kPosTolerance = 1e-4;   // 0.1 mm
constexpr double kStepTolerance = 1e-6;  // rad
constexpr double kMaxStep = 0.10;        // rad per iteration
constexpr double kLambdaMin = 1e-3;
constexpr double kLambdaMax = 0.5;

inline double clampJoint(std::size_t i, double q) {
    return std::min(std::max(q, kTable[i].q_min), kTable[i].q_max);
}

inline double wrapPi(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

// Solve a small dense system via Gauss-Jordan with partial pivot. n is at most
// 3 here (position-only task), so clarity beats pulling in LAPACK.
bool solveDense(std::array<std::array<double, 3>, 3>& A,
                std::array<double, 3>& b, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int row = col + 1; row < n; ++row) {
            if (std::fabs(A[row][col]) > std::fabs(A[pivot][col])) pivot = row;
        }
        if (std::fabs(A[pivot][col]) < 1e-12) return false;
        if (pivot != col) {
            std::swap(A[pivot], A[col]);
            std::swap(b[pivot], b[col]);
        }
        const double inv = 1.0 / A[col][col];
        for (int k = col; k < n; ++k) A[col][k] *= inv;
        b[col] *= inv;
        for (int row = 0; row < n; ++row) {
            if (row == col) continue;
            const double f = A[row][col];
            if (f == 0.0) continue;
            for (int k = col; k < n; ++k) A[row][k] -= f * A[col][k];
            b[row] -= f * b[col];
        }
    }
    return true;
}

// Single-row standard DH transform.
Transform4 dhTransform(const DhParams& row, double q) {
    const double th = row.theta_offset + q;
    const double ct = std::cos(th), st = std::sin(th);
    const double ca = std::cos(row.alpha), sa = std::sin(row.alpha);

    Transform4 t;
    t.m = {ct, -st * ca,  st * sa, row.a * ct,
           st,  ct * ca, -ct * sa, row.a * st,
           0.0, sa,       ca,      row.d,
           0.0, 0.0,      0.0,     1.0};
    return t;
}

}  // namespace

Transform4 Transform4::identity() {
    Transform4 t;
    t.m = {1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1};
    return t;
}

Transform4 Transform4::operator*(const Transform4& rhs) const {
    Transform4 out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double acc = 0.0;
            for (int k = 0; k < 4; ++k) acc += m[r * 4 + k] * rhs.m[k * 4 + c];
            out.m[r * 4 + c] = acc;
        }
    }
    return out;
}

std::array<double, 3> Transform4::rotate(const std::array<double, 3>& v) const {
    return {
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
        m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
    };
}

// Forward kinematics. Returns the tool pose in the base frame.
Pose forwardKinematics(const JointConfig& qc) {
    Transform4 T = Transform4::identity();
    for (std::size_t i = 0; i < kDof; ++i) {
        T = T * dhTransform(kTable[i], clampJoint(i, qc[i]));
    }

    Pose out;
    out.p = {T.m[3], T.m[7], T.m[11]};
    out.rot = T;
    return out;
}

// Numerical Jacobian, position rows only (3x4). Step size 1e-6 rad keeps the
// finite difference well above the quantisation of the 14-bit AS5047P.
void positionJacobian(const JointConfig& qc,
                      std::array<std::array<double, kDof>, 3>& J) {
    constexpr double h = 1e-6;
    const Pose base = forwardKinematics(qc);

    for (std::size_t j = 0; j < kDof; ++j) {
        JointConfig probe = qc;
        probe[j] += h;
        const Pose shifted = forwardKinematics(probe);
        for (std::size_t r = 0; r < 3; ++r) {
            J[r][j] = (shifted.p[r] - base.p[r]) / h;
        }
    }
}

// Damped least squares IK. Position-only task; wrist roll is left where the
// operator put it during calibration.
bool inverseKinematicsDls(JointConfig& qc,
                          const std::array<double, 3>& target,
                          int* iterations_used) {
    double lambda = kLambdaMin;

    for (int iter = 0; iter < kMaxIterations; ++iter) {
        const Pose current = forwardKinematics(qc);

        std::array<double, 3> err = {
            target[0] - current.p[0],
            target[1] - current.p[1],
            target[2] - current.p[2],
        };
        const double err_norm =
            std::sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);

        if (err_norm < kPosTolerance) {
            if (iterations_used) *iterations_used = iter;
            return true;
        }

        std::array<std::array<double, kDof>, 3> J{};
        positionJacobian(qc, J);

        // A = J J^T + lambda^2 I  (3x3, symmetric positive definite)
        std::array<std::array<double, 3>, 3> A{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                double acc = 0.0;
                for (std::size_t k = 0; k < kDof; ++k) acc += J[r][k] * J[c][k];
                A[r][c] = acc;
            }
            A[r][r] += lambda * lambda;
        }

        std::array<double, 3> rhs = err;
        if (!solveDense(A, rhs, 3)) {
            // Rank-deficient even with damping: widen it and retry next pass.
            lambda = std::min(lambda * 4.0, kLambdaMax);
            continue;
        }

        double max_step = 0.0;
        for (std::size_t j = 0; j < kDof; ++j) {
            double dq = 0.0;
            for (int r = 0; r < 3; ++r) dq += J[r][j] * rhs[r];

            // Hard cap per iteration so a bad Jacobian cannot fling the arm.
            dq = std::min(std::max(dq, -kMaxStep), kMaxStep);
            const double next = clampJoint(j, wrapPi(qc[j] + dq));
            const double applied = next - qc[j];
            qc[j] = next;
            max_step = std::max(max_step, std::fabs(applied));
        }

        // Singularity damping schedule: grow lambda when progress stalls,
        // shrink it again once the step is healthy.
        if (max_step < 1e-4) {
            lambda = std::min(lambda * 2.0, kLambdaMax);
        } else {
            lambda = std::max(lambda * 0.85, kLambdaMin);
        }

        if (max_step < kStepTolerance) {
            // Step size collapsed but the error is still outside tolerance:
            // we are pinned against a joint limit or a singularity.
            if (iterations_used) *iterations_used = iter;
            return false;
        }
    }

    if (iterations_used) *iterations_used = kMaxIterations;
    return false;
}

}  // namespace kin

// Small self-check: drive the arm from a known pose to a reachable target.
int main() {
    kin::JointConfig qc;
    qc.q = {0.0, 0.60, -0.90, 0.0};

    const kin::Pose start = kin::forwardKinematics(qc);
    std::printf("start  p = % .4f % .4f % .4f\n",
                start.p[0], start.p[1], start.p[2]);

    const std::array<double, 3> target = {0.180, 0.240, 0.310};
    int iters = 0;
    const bool ok = kin::inverseKinematicsDls(qc, target, &iters);

    const kin::Pose end = kin::forwardKinematics(qc);
    std::printf("ik     ok=%d iters=%d\n", ok ? 1 : 0, iters);
    std::printf("end    p = % .4f % .4f % .4f\n",
                end.p[0], end.p[1], end.p[2]);
    std::printf("target p = % .4f % .4f % .4f\n",
                target[0], target[1], target[2]);
    std::printf("q      = % .4f % .4f % .4f % .4f\n",
                qc[0], qc[1], qc[2], qc[3]);
    return ok ? 0 : 1;
}

// TODO(ravi): expose the lambda schedule and per-joint weights through a config
// struct so the gripper-heavy payload variant can de-weight the wrist roll.
