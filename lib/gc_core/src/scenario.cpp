#include "scenario.h"
#include <cmath>
#include <string.h>

namespace gc {

void default_setup(RobotParams& rp, CtrlParams& cp, MetricParams& mp, real q0[NJ], real q_des[NJ]) {
  rp = RobotParams();
  cp = CtrlParams();
  mp = MetricParams();
  // Start/goal of the MATLAB simulation: mirrored pan angle, the straight joint-space
  // path swings both links through the excluded wedge.
  q0[0] = real(1.2);  q0[1] = real(0.5); q0[2] = real(-0.2);
  q_des[0] = real(-1.2); q_des[1] = real(0.5); q_des[2] = real(-0.2);
}

void disturbance(real t, real d[NJ]) {
  d[0] = 5 * std::sin(3 * t);
  d[1] = 10 * std::cos(t);
  d[2] = -10 * std::sin(real(1.5) * t);
}

real outside_depth(const V3r& x) {
  // Must match build_wedge_workspace(): box [-0.5,2.8] x [-2,2] x [-1,2], wedge {x >= 0.8 + |y|}.
  real dep = 0;
  const real dw = (x.x - real(0.8) - std::fabs(x.y)) * real(0.70710678);  // inside wedge
  if (dw > dep) dep = dw;
  const real db[6] = {real(-0.5) - x.x, x.x - real(2.8), -2 - x.y, x.y - 2, -1 - x.z, x.z - 2};
  for (real v : db) if (v > dep) dep = v;
  return dep;
}

static void plant_rk4(const RobotParams& rp, real q[NJ], real qd[NJ], const real u[NJ],
                      bool dist, real t, real dt, const CtrlParams& cp) {
  real us[NJ];
  for (int i = 0; i < NJ; ++i) us[i] = u[i] > cp.u_max[i] ? cp.u_max[i] : (u[i] < cp.u_min[i] ? cp.u_min[i] : u[i]);
  real k1q[NJ], k1v[NJ], k2q[NJ], k2v[NJ], k3q[NJ], k3v[NJ], k4q[NJ], k4v[NJ];
  real qt[NJ], vt[NJ], d[NJ] = {0, 0, 0};
  if (dist) disturbance(t, d);
  forward_dyn(rp, q, qd, us, d, k1v);
  for (int i = 0; i < NJ; ++i) { k1q[i] = qd[i]; qt[i] = q[i] + real(0.5) * dt * k1q[i]; vt[i] = qd[i] + real(0.5) * dt * k1v[i]; }
  if (dist) disturbance(t + real(0.5) * dt, d);
  forward_dyn(rp, qt, vt, us, d, k2v);
  for (int i = 0; i < NJ; ++i) { k2q[i] = vt[i]; qt[i] = q[i] + real(0.5) * dt * k2q[i]; vt[i] = qd[i] + real(0.5) * dt * k2v[i]; }
  forward_dyn(rp, qt, vt, us, d, k3v);
  for (int i = 0; i < NJ; ++i) { k3q[i] = vt[i]; qt[i] = q[i] + dt * k3q[i]; vt[i] = qd[i] + dt * k3v[i]; }
  if (dist) disturbance(t + dt, d);
  forward_dyn(rp, qt, vt, us, d, k4v);
  for (int i = 0; i < NJ; ++i) { k4q[i] = vt[i];
    q[i] += dt / 6 * (k1q[i] + 2 * k2q[i] + 2 * k3q[i] + k4q[i]);
    qd[i] += dt / 6 * (k1v[i] + 2 * k2v[i] + 2 * k3v[i] + k4v[i]); }
}

void run_scenario(const ScenarioCfg& cfg, ClockFn clock, real ticks_per_us,
                  LogFn log, void* user, BenchStats& out) {
  static Workspace ws;     // static: keep large objects off the (small) MCU task stack
  static Controller ctl;
  RobotParams rp; CtrlParams cp; MetricParams mp;
  real q[NJ], qd[NJ] = {0, 0, 0}, q_des[NJ];
  default_setup(rp, cp, mp, q, q_des);
  if (cfg.tweak) cfg.tweak(rp, cp, mp);
  cp.enable_cbf = cfg.cbf;
  build_wedge_workspace(ws, mp);
  ctl = Controller();
  ctl.init(rp, cp, mp, ws, q, qd);
  ctl.clock = clock;

  uint32_t* hist = out.hist;
  memset(&out, 0, sizeof(out));
  out.hist = hist;
  if (hist) memset(hist, 0, sizeof(uint32_t) * HIST_BINS);
  out.ticks_per_us = ticks_per_us;
  out.min_h = 1e9f;

  const int steps = (int)(cfg.T / cfg.dt + real(0.5));
  real u[NJ];
  for (int k = 0; k < steps; ++k) {
    const real t = k * cfg.dt;
    ctl.step(q, qd, q_des, cfg.dt, u);

    for (int s = 0; s < NSTAGE; ++s) {
      out.sum[s] += ctl.ticks[s];
      if (ctl.ticks[s] > out.max[s]) out.max[s] = ctl.ticks[s];
    }
    if (hist && ticks_per_us > 0) {
      uint32_t b = (uint32_t)(ctl.ticks[ST_TOTAL] / ticks_per_us);
      hist[b < HIST_BINS ? b : HIST_BINS - 1]++;
    }
    out.qp_iter_sum += ctl.qp_iters;
    if ((uint32_t)ctl.qp_iters > out.qp_iter_max) out.qp_iter_max = ctl.qp_iters;
    out.fallbacks += ctl.fallback;
    out.n_con = ctl.n_con;
    ++out.steps;

    // Ground-truth safety check (independent of whether the filter is enabled).
    {
      V3r p[NL + 1];
      fk(rp, q, p);
      real pen = 0;
      for (int i = 0; i < NL; ++i) {
        for (int f = 0; f < ws.nf; ++f) {
          const real hv = pair_metric(p[i], p[i + 1], ws.f[f], mp);
          if (hv < out.min_h) out.min_h = hv;
        }
        for (int sidx = 0; sidx <= 20; ++sidx) {   // sample the link densely
          const real dp = outside_depth(p[i] + scale(real(sidx) / 20, p[i + 1] - p[i]));
          if (dp > pen) pen = dp;
        }
      }
      if (pen > 0) { out.violations++; if (pen > out.max_pen) out.max_pen = pen; }
      if (log && cfg.log_every > 0 && k % cfg.log_every == 0) {
        LogRow r;
        r.t = t;
        real d[NJ] = {0, 0, 0};
        if (cfg.disturbance) disturbance(t, d);
        for (int i = 0; i < NJ; ++i) { r.q[i] = q[i]; r.u[i] = u[i]; r.dhat[i] = ctl.dhat[i]; r.d[i] = d[i]; }
        r.h_w1 = pair_metric(p[1], p[2], ws.f[ws.nf - 2], mp);
        r.h_w2 = pair_metric(p[1], p[2], ws.f[ws.nf - 1], mp);
        real hm = 1e9f;
        for (int i = 0; i < NL; ++i)
          for (int f = 0; f < ws.nf; ++f) { real hv = pair_metric(p[i], p[i + 1], ws.f[f], mp); if (hv < hm) hm = hv; }
        r.h_min = hm;
        r.qp_iters = ctl.qp_iters;
        r.fallback = ctl.fallback;
        r.pen_mm = pen > 0 ? pen * 1000 : 0;
        r.ctrl_us = ticks_per_us > 0 ? ctl.ticks[ST_TOTAL] / ticks_per_us : 0;
        r.p2[0] = p[2].x; r.p2[1] = p[2].y; r.p2[2] = p[2].z;
        log(r, user);
      }
    }

    plant_rk4(rp, q, qd, u, cfg.disturbance, t, cfg.dt, cp);
  }
  real e = 0;
  for (int i = 0; i < NJ; ++i) e += (q[i] - q_des[i]) * (q[i] - q_des[i]);
  out.final_err = std::sqrt(e);
}

real percentile_us(const BenchStats& s, real pct) {
  if (!s.hist || s.steps == 0) return 0;
  const uint32_t target = (uint32_t)(pct / 100 * s.steps);
  uint32_t acc = 0;
  for (int b = 0; b < HIST_BINS; ++b) {
    acc += s.hist[b];
    if (acc > target) return (real)b + real(0.5);
  }
  return HIST_BINS;
}

}  // namespace gc
