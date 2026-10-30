#include "controller.h"
#include <cmath>

namespace gc {

static inline uint32_t tick(ClockFn c) { return c ? c() : 0; }

void Controller::init(const RobotParams& rp, const CtrlParams& cp, const MetricParams& mp,
                      const Workspace& ws, const real q0[NJ], const real qd0[NJ]) {
  rp_ = rp; cp_ = cp; mp_ = mp; ws_ = &ws;
  Dyn D;
  compute_dyn(rp_, q0, qd0, D);
  for (int i = 0; i < NJ; ++i) {               // z(0) chosen so that dhat(0) = 0
    real Mq = 0;
    for (int j = 0; j < NJ; ++j) Mq += D.M[i][j] * qd0[j];
    z_[i] = -cp_.Kd[i] * Mq;
  }
  for (int i = 0; i < MAX_CON; ++i) lambda_[i] = 0;
}

void Controller::step(const real q[NJ], const real qd[NJ], const real q_des[NJ], real dt, real u[NJ]) {
  const uint32_t t0 = tick(clock);

  // ---- 1) dynamics terms -------------------------------------------------
  Dyn D;
  compute_dyn(rp_, q, qd, D);
  real Cqd[NJ];
  for (int i = 0; i < NJ; ++i) {
    Cqd[i] = 0;
    for (int j = 0; j < NJ; ++j) Cqd[i] += D.C[i][j] * qd[j];
  }
  const uint32_t t1 = tick(clock);

  // ---- 2) disturbance observer + nominal PD-like law -----------------------
  for (int i = 0; i < NJ; ++i) {
    real Mq = 0;
    for (int j = 0; j < NJ; ++j) Mq += D.M[i][j] * qd[j];
    dhat[i] = cp_.enable_dob ? z_[i] + cp_.Kd[i] * Mq : 0;          // Eq. (3)
  }
  real unom[NJ];
  for (int i = 0; i < NJ; ++i)
    unom[i] = -cp_.D[i] * qd[i] - cp_.P[i] * (q[i] - q_des[i]) + D.G[i]   // Eq. (6)
              - (cp_.nominal_dob_ff ? dhat[i] : real(0));

  // Drift acceleration a0 = M^-1 (dhat - C qd - G - K qd); qdd = a0 + M^-1 u.
  real f[NJ], a0[NJ];
  for (int i = 0; i < NJ; ++i) f[i] = dhat[i] - Cqd[i] - D.G[i] - rp_.kf[i] * qd[i];
  for (int i = 0; i < NJ; ++i) {
    a0[i] = 0;
    for (int j = 0; j < NJ; ++j) a0[i] += D.Minv[i][j] * f[j];
  }
  const uint32_t t2 = tick(clock);

  // ---- 3) safety metrics + HOCBF rows (forward-mode AD, one pass per pair) --
  int m = 0;
  h_min = 1e9f;
  if (cp_.enable_cbf) {
    Jet qj[NJ];
    for (int k = 0; k < NJ; ++k) {
      qj[k] = Jet(q[k]);
      qj[k].g[k] = 1;
      qj[k].d = qd[k];
    }
    V3<Jet> p[NL + 1];
    fk(rp_, qj, p);
    const real g1 = cp_.gamma1, g2 = cp_.gamma2;
    for (int i = 0; i < NL; ++i) {
      for (int k = 0; k < ws_->nf; ++k) {
        const Jet hk = pair_metric(p[i], p[i + 1], ws_->f[k], mp_);
        h[i][k] = hk.v;
        if (hk.v < h_min) h_min = hk.v;
        if (hk.v >= cp_.h_activate) {   // inactive pair: zero row keeps indices fixed
          for (int c = 0; c < NJ; ++c) A_[m][c] = 0;
          b_[m] = 0;
          lambda_[m] = 0;
          ++m;
          continue;
        }
        // A = -J M^-1,  b = Jdot qd + (g1+g2) hdot + g1 g2 h + J a0   (Eq. 15)
        real row[NJ], JA = 0;
        for (int c = 0; c < NJ; ++c) {
          row[c] = 0;
          for (int r = 0; r < NJ; ++r) row[c] -= hk.g[r] * D.Minv[r][c];
          JA += hk.g[c] * a0[c];
        }
        real bb = hk.s + (g1 + g2) * hk.d + g1 * g2 * hk.v + JA;
        const real nr = std::sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);
        if (nr < real(1e-9)) {
          // Metric locally flat: the row cannot be influenced by u. Keep a zero row so
          // the warm-start indices stay fixed; if bb < 0 it is infeasible -> fallback.
          for (int c = 0; c < NJ; ++c) A_[m][c] = 0;
          b_[m] = bb < 0 ? bb : 0;
        } else {
          for (int c = 0; c < NJ; ++c) A_[m][c] = row[c] / nr;
          b_[m] = bb / nr;
        }
        ++m;
      }
    }
    // Velocity CBFs (Eq. 18): +-M^-1_i u <= gamma h_vel -+ a0_i
    for (int i = 0; i < NJ; ++i) {
      real nr = std::sqrt(D.Minv[i][0] * D.Minv[i][0] + D.Minv[i][1] * D.Minv[i][1] + D.Minv[i][2] * D.Minv[i][2]);
      for (int c = 0; c < NJ; ++c) A_[m][c] = D.Minv[i][c] / nr;
      b_[m++] = (cp_.gamma_vel * (cp_.qd_max[i] - qd[i]) - a0[i]) / nr;
      for (int c = 0; c < NJ; ++c) A_[m][c] = -D.Minv[i][c] / nr;
      b_[m++] = (cp_.gamma_vel * (qd[i] - cp_.qd_min[i]) + a0[i]) / nr;
    }
  }
  // Torque bounds.
  for (int i = 0; i < NJ; ++i) {
    for (int c = 0; c < NJ; ++c) { A_[m][c] = (c == i); A_[m + 1][c] = -real(c == i); }
    b_[m] = cp_.u_max[i];
    b_[m + 1] = -cp_.u_min[i];
    m += 2;
  }
  n_con = m;
  const uint32_t t3 = tick(clock);

  // ---- 4) QP safety filter, braking fallback on failure --------------------
  const QPResult r = qp_hildreth(A_, b_, m, unom, lambda_, u, cp_.qp_max_iter, cp_.qp_tol);
  qp_iters = r.iters;
  fallback = !r.ok;
  if (fallback) {                                                       // Eq. (20)
    for (int i = 0; i < NJ; ++i) {
      u[i] = Cqd[i] + D.G[i] + rp_.kf[i] * qd[i] - dhat[i] - cp_.Kbrake[i] * qd[i];
    }
    for (int i = 0; i < MAX_CON; ++i) lambda_[i] = 0;
  }

  // DOB state update (Eq. 2), explicit Euler, using the torque actually applied.
  if (cp_.enable_dob) {
    for (int i = 0; i < NJ; ++i) {
      real CTq = 0;  // C^T qd
      for (int j = 0; j < NJ; ++j) CTq += D.C[j][i] * qd[j];
      const real KCq = rp_.kf[i] * qd[i] - CTq;
      z_[i] += dt * cp_.Kd[i] * (KCq + D.G[i] - u[i] - dhat[i]);
    }
  }
  const uint32_t t4 = tick(clock);

  ticks[ST_DYN] = t1 - t0;
  ticks[ST_DOB_NOM] = t2 - t1;
  ticks[ST_SAFETY] = t3 - t2;
  ticks[ST_QP] = t4 - t3;
  ticks[ST_TOTAL] = t4 - t0;
}

}  // namespace gc
