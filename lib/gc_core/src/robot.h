// robot.h - 3-DOF serial manipulator of Sec. IV (pan q1, shoulder elevation q2, elbow q3).
#pragma once
#include "vec3.h"

namespace gc {

struct RobotParams {
  real Ib = 0.5f, Iy1 = 0.1f, Iy2 = 0.1f;  // Table I
  real m1 = 1.0f, m2 = 1.0f;
  real L1 = 0.9f, L2 = 0.9f;
  real r1 = 0.45f, r2 = 0.45f;
  real g = 9.81f;
  real kf[NJ] = {2.0f, 2.0f, 2.0f};        // viscous friction K
};

struct Dyn {
  real M[NJ][NJ], C[NJ][NJ], G[NJ], Minv[NJ][NJ];
};

// Inertia, Coriolis (Christoffel form), gravity, and M^-1 for the elevation convention.
void compute_dyn(const RobotParams& P, const real q[NJ], const real qd[NJ], Dyn& D);

// qdd = M^-1 (u + d - C qd - G - K qd)
void forward_dyn(const RobotParams& P, const real q[NJ], const real qd[NJ],
                 const real u[NJ], const real d[NJ], real qdd[NJ]);

// Forward kinematics of the link endpoints p0 (base), p1 (elbow), p2 (tip).
// q1 = base pan about z, q2 = shoulder elevation measured from the horizontal,
// q3 = elbow angle relative to link 1 (same convention as the MATLAB simulation).
template <class S>
void fk(const RobotParams& P, const S q[NJ], V3<S> p[NL + 1]) {
  S s1 = m_sin(q[0]), c1 = m_cos(q[0]);
  S s2 = m_sin(q[1]), c2 = m_cos(q[1]);
  S q23 = q[1] + q[2];
  S s23 = m_sin(q23), c23 = m_cos(q23);
  S rho1 = P.L1 * c2;              // horizontal reach of p1
  S rho2 = rho1 + P.L2 * c23;      // horizontal reach of p2
  S z1 = P.L1 * s2;
  p[0] = {S(0), S(0), S(0)};
  p[1] = {rho1 * c1, rho1 * s1, z1};
  p[2] = {rho2 * c1, rho2 * s1, z1 + P.L2 * s23};
}

}  // namespace gc
