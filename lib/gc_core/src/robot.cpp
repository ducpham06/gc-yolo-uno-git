#include "robot.h"

namespace gc {

static void inv3(const real A[NJ][NJ], real R[NJ][NJ]) {
  real c00 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
  real c01 = A[1][2] * A[2][0] - A[1][0] * A[2][2];
  real c02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
  real det = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02;
  real id = real(1) / det;
  R[0][0] = c00 * id;
  R[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * id;
  R[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * id;
  R[1][0] = c01 * id;
  R[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * id;
  R[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * id;
  R[2][0] = c02 * id;
  R[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * id;
  R[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * id;
}

void compute_dyn(const RobotParams& P, const real q[NJ], const real qd[NJ], Dyn& D) {
  const real s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const real s3 = std::sin(q[2]), c3 = std::cos(q[2]);
  const real s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  // Horizontal reaches (elevation convention): link-1 CoM r1 cos q2, link-2 CoM A.
  const real A = P.L1 * c2 + P.r2 * c23;
  const real B = P.L1 * s2 + P.r2 * s23;  // = -dA/dq2

  const real M11 = P.Ib + P.m1 * (P.r1 * c2) * (P.r1 * c2) + P.m2 * A * A;
  const real M22 = P.Iy1 + P.Iy2 + P.m1 * P.r1 * P.r1 +
                   P.m2 * (P.L1 * P.L1 + P.r2 * P.r2 + 2 * P.L1 * P.r2 * c3);
  const real M23 = P.Iy2 + P.m2 * (P.r2 * P.r2 + P.L1 * P.r2 * c3);
  const real M33 = P.Iy2 + P.m2 * P.r2 * P.r2;
  const real h = P.m2 * P.L1 * P.r2 * s3;
  const real dM11_2 = -2 * P.m1 * P.r1 * P.r1 * s2 * c2 - 2 * P.m2 * A * B;
  const real dM11_3 = -2 * P.m2 * A * P.r2 * s23;

  D.M[0][0] = M11; D.M[0][1] = 0;   D.M[0][2] = 0;
  D.M[1][0] = 0;   D.M[1][1] = M22; D.M[1][2] = M23;
  D.M[2][0] = 0;   D.M[2][1] = M23; D.M[2][2] = M33;

  D.C[0][0] = real(0.5) * (dM11_2 * qd[1] + dM11_3 * qd[2]);
  D.C[0][1] = real(0.5) * dM11_2 * qd[0];
  D.C[0][2] = real(0.5) * dM11_3 * qd[0];
  D.C[1][0] = -real(0.5) * dM11_2 * qd[0];
  D.C[1][1] = -h * qd[2];
  D.C[1][2] = -h * (qd[1] + qd[2]);
  D.C[2][0] = -real(0.5) * dM11_3 * qd[0];
  D.C[2][1] = h * qd[1];
  D.C[2][2] = 0;

  // Gravity G = dV/dq with heights z = r1 sin q2 (link 1), L1 sin q2 + r2 sin(q2+q3) (link 2).
  D.G[0] = 0;
  D.G[1] = (P.m1 * P.r1 + P.m2 * P.L1) * P.g * c2 + P.m2 * P.r2 * P.g * c23;
  D.G[2] = P.m2 * P.r2 * P.g * c23;

  inv3(D.M, D.Minv);
}

void forward_dyn(const RobotParams& P, const real q[NJ], const real qd[NJ],
                 const real u[NJ], const real d[NJ], real qdd[NJ]) {
  Dyn D;
  compute_dyn(P, q, qd, D);
  real f[NJ];
  for (int i = 0; i < NJ; ++i) {
    real Cq = 0;
    for (int j = 0; j < NJ; ++j) Cq += D.C[i][j] * qd[j];
    f[i] = u[i] + d[i] - Cq - D.G[i] - P.kf[i] * qd[i];
  }
  for (int i = 0; i < NJ; ++i) {
    qdd[i] = 0;
    for (int j = 0; j < NJ; ++j) qdd[i] += D.Minv[i][j] * f[j];
  }
}

}  // namespace gc
