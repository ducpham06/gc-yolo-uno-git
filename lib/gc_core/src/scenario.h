// scenario.h - closed-loop "processor-in-the-loop" run: the controller runs on the target,
// the manipulator is simulated on the same chip (RK4), and only the controller is timed.
#pragma once
#include "controller.h"

namespace gc {

struct ScenarioCfg {
  const char* name = "ideal";
  bool cbf = true;          // false -> nominal controller only (shows the violation)
  bool disturbance = false; // d = [5 sin 3t, 10 cos t, -10 sin 1.5t]
  real T = 5.0f;            // duration [s]
  real dt = 0.001f;         // control period [s] (1 kHz)
  int log_every = 10;       // emit a log row every N steps (0 = never)
  // Optional hook to override the defaults (gains, alpha, ...) before the run.
  void (*tweak)(RobotParams&, CtrlParams&, MetricParams&) = nullptr;
};

struct LogRow {
  real t, q[NJ], u[NJ], dhat[NJ], d[NJ];
  real h_w1, h_w2;          // link 2 vs wedge plane 1 / plane 2 (Fig. 5)
  real h_min;               // min over all (link, face) pairs
  int qp_iters, fallback;
  real pen_mm;              // exact penetration depth this step [mm] (0 = contained)
  real ctrl_us;             // controller time this step
  real p2[3];               // tip position
};

constexpr int HIST_BINS = 4096;  // 1 us bins for percentile estimates

struct BenchStats {
  uint64_t sum[NSTAGE];
  uint32_t max[NSTAGE];
  uint32_t steps;
  uint32_t qp_iter_sum, qp_iter_max, fallbacks;
  int n_con;
  real min_h;               // worst smoothed metric over the run (true state)
  uint32_t violations;      // steps where some point of the arm is outside P (exact, ground truth)
  real max_pen;             // worst penetration depth into the excluded region [m]
  real final_err;           // ||q(T) - q_des||
  real ticks_per_us;
  uint32_t* hist;           // HIST_BINS counters, provided by caller
};

typedef void (*LogFn)(const LogRow& row, void* user);

void default_setup(RobotParams& rp, CtrlParams& cp, MetricParams& mp, real q0[NJ], real q_des[NJ]);
void disturbance(real t, real d[NJ]);
// Exact depth of point x outside the admissible set P of build_wedge_workspace() (0 if inside).
real outside_depth(const V3r& x);
void run_scenario(const ScenarioCfg& cfg, ClockFn clock, real ticks_per_us,
                  LogFn log, void* user, BenchStats& out);
// Percentile (0..100) of total controller time in microseconds, from the histogram.
real percentile_us(const BenchStats& s, real pct);

}  // namespace gc
