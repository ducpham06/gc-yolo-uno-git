// Host build of the exact same controller core: functional check + PC timing baseline.
//   make -C host && ./host/gc_host            (writes host/log_<scenario>.csv)
#include <chrono>
#include <cstdio>
#include "scenario.h"

using namespace gc;

static uint32_t clock_ns() {
  using namespace std::chrono;
  return (uint32_t)duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

static void log_csv(const LogRow& r, void* user) {
  FILE* f = (FILE*)user;
  fprintf(f, "%.4f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.2f,%.4f,%.4f,%.4f\n",
          r.t, r.q[0], r.q[1], r.q[2], r.u[0], r.u[1], r.u[2], r.dhat[0], r.dhat[1], r.dhat[2],
          r.d[0], r.d[1], r.d[2], r.h_w1, r.h_w2, r.h_min, r.qp_iters, r.fallback, r.pen_mm, r.ctrl_us,
          r.p2[0], r.p2[1], r.p2[2]);
}

int main() {
  static uint32_t hist[HIST_BINS];
  ScenarioCfg cfgs[3];
  cfgs[0].name = "nominal_only"; cfgs[0].cbf = false;
  cfgs[1].name = "ideal";
  cfgs[2].name = "disturbed";    cfgs[2].disturbance = true;
  printf("scalar=%s bytes=%d\n", sizeof(real) == 4 ? "float" : "double", (int)sizeof(real));
  for (auto& c : cfgs) {
    char fn[64];
    snprintf(fn, sizeof fn, "host/log_%s.csv", c.name);
    FILE* f = fopen(fn, "w");
    fprintf(f, "t,q1,q2,q3,u1,u2,u3,dh1,dh2,dh3,d1,d2,d3,h_w1,h_w2,h_min,qp_iters,fallback,pen_mm,ctrl_us,x2,y2,z2\n");
    BenchStats s; s.hist = hist;
    run_scenario(c, clock_ns, 1000.0f, log_csv, f, s);
    fclose(f);
    printf("%-13s min_h=%+.3f viol_steps=%u max_pen=%.1fmm final_err=%.4f rad  fallbacks=%u  qp_iters avg=%.1f max=%u  m=%d  "
           "ctrl avg=%.2f us p99=%.1f us max=%.1f us\n",
           c.name, s.min_h, s.violations, s.max_pen*1000, s.final_err, s.fallbacks, (double)s.qp_iter_sum / s.steps, s.qp_iter_max,
           s.n_con, (double)s.sum[ST_TOTAL] / s.steps / 1000.0, percentile_us(s, 99),
           s.max[ST_TOTAL] / 1000.0);
  }
  return 0;
}
