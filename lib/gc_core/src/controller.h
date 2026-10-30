// controller.h - DOB (Eqs. 2-3) + PD-like nominal (Eq. 6) + HOCBF/velocity-CBF QP filter
// (Eqs. 15, 18, 19) + dissipative braking fallback (Eq. 20).
#pragma once
#include "qp.h"
#include "safety.h"

namespace gc {

struct CtrlParams {
  // Defaults follow the authors' MATLAB simulation, except gamma_vel (15 -> 50) and the
  // DOB gain (40 -> 200), which the full dynamic model needs (see README).
  real P[NJ] = {50, 50, 50};        // proportional gains (Kp)
  real D[NJ] = {10, 10, 10};        // derivative gains (Kd)
  real Kd[NJ] = {200, 200, 200};    // DOB gains
  real Kbrake[NJ] = {150, 150, 150};// fallback damping
  real gamma1 = 10, gamma2 = 10;    // HOCBF class-K gains
  real gamma_vel = 50;              // velocity CBF gain (low values cap deceleration -> QP conflicts)
  real qd_max[NJ] = {1.5f, 1.5f, 1.5f};
  real qd_min[NJ] = {-1.5f, -1.5f, -1.5f};
  real u_max[NJ] = {200, 200, 200}; // Table I torque limit
  real u_min[NJ] = {-200, -200, -200};
  // HOCBF rows are only built for pairs with h_ik < h_activate. When link i does not
  // straddle plane k (h1 ~ +1) the pair is safe (h >= c ~ 1) whatever h2 does, yet h2 can
  // swing hard as the *extrapolated* intersection point crosses face edges; constraining
  // those harmless swings makes the QP ill-conditioned. Set to 1e9 for the paper's
  // "all pairs always" formulation.
  real h_activate = 1.2f;
  bool enable_cbf = true;
  bool enable_dob = true;
  // Adds -dhat to the nominal law, as in the MATLAB code. Eq. (6) in the paper omits it;
  // without it a disturbance leaves a steady-state error of about P^-1 d.
  bool nominal_dob_ff = true;
  int qp_max_iter = 300;
  real qp_tol = 1e-4f;
};

typedef uint32_t (*ClockFn)();  // monotonically increasing tick counter (cycles, ns, ...)

class Controller {
 public:
  void init(const RobotParams& rp, const CtrlParams& cp, const MetricParams& mp,
            const Workspace& ws, const real q0[NJ], const real qd0[NJ]);
  // One control period. Writes the torque command to u.
  void step(const real q[NJ], const real qd[NJ], const real q_des[NJ], real dt, real u[NJ]);

  ClockFn clock = nullptr;      // optional profiler hook
  uint32_t ticks[NSTAGE] = {};  // ticks spent per stage in the last step

  // Diagnostics from the last step.
  real dhat[NJ] = {};
  real h[NL][MAX_FACES] = {};
  real h_min = 0;
  int qp_iters = 0;
  bool fallback = false;
  int n_con = 0;

 private:
  RobotParams rp_;
  CtrlParams cp_;
  MetricParams mp_;
  const Workspace* ws_ = nullptr;
  real z_[NJ] = {};
  real lambda_[MAX_CON] = {};
  real A_[MAX_CON][NJ];
  real b_[MAX_CON];
};

}  // namespace gc
