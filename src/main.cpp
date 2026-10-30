// Yolo UNO (ESP32-S3) firmware: processor-in-the-loop benchmark of the geometric
// containment controller (DOB + PD-like + HOCBF-QP + braking fallback).
//
// The controller runs exactly as it would on a real arm; the 3-DOF manipulator is
// simulated on the same chip (RK4) so the loop is closed. Only the controller is timed,
// using the CPU cycle counter. Results stream over USB serial as CSV blocks:
//   #BEGIN <scenario> / CSV header / rows / #SUMMARY <scenario> k=v ... / #END <scenario>
// Use tools/capture_serial.py to save and plot them.
//
// Serial commands:  r = rerun all scenarios with logs,  b = benchmark only (no logs, 5 runs each)
#include <Arduino.h>
#include "scenario.h"

using namespace gc;

static uint32_t hist[HIST_BINS];   // static: keep 16 KB off the loop-task stack

static uint32_t clock_cycles() { return ESP.getCycleCount(); }

static void log_row(const LogRow& r, void*) {
  Serial.printf("%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                (double)r.t, (double)r.q[0], (double)r.q[1], (double)r.q[2],
                (double)r.u[0], (double)r.u[1], (double)r.u[2],
                (double)r.dhat[0], (double)r.dhat[1], (double)r.dhat[2],
                (double)r.d[0], (double)r.d[1], (double)r.d[2],
                (double)r.h_w1, (double)r.h_w2, (double)r.h_min, r.qp_iters, r.fallback,
                (double)r.pen_mm, (double)r.ctrl_us, (double)r.p2[0], (double)r.p2[1], (double)r.p2[2]);
}

static void print_summary(const char* name, const BenchStats& s, real dt) {
  const double tpu = s.ticks_per_us;
  auto avg = [&](int st) { return (double)s.sum[st] / s.steps / tpu; };
  auto mx = [&](int st) { return (double)s.max[st] / tpu; };
  Serial.printf("#SUMMARY %s scalar=%s steps=%u n_con=%d "
                "dyn_avg_us=%.2f dob_nom_avg_us=%.2f safety_avg_us=%.2f qp_avg_us=%.2f total_avg_us=%.2f "
                "safety_max_us=%.1f qp_max_us=%.1f total_max_us=%.1f total_p50_us=%.1f total_p99_us=%.1f "
                "budget_us=%.0f util_avg_pct=%.2f util_max_pct=%.1f "
                "qp_iter_avg=%.2f qp_iter_max=%u fallbacks=%u min_h=%.4f viol_steps=%u max_pen_mm=%.2f final_err_rad=%.4f\n",
                name, sizeof(real) == 4 ? "float" : "double", s.steps, s.n_con,
                avg(ST_DYN), avg(ST_DOB_NOM), avg(ST_SAFETY), avg(ST_QP), avg(ST_TOTAL),
                mx(ST_SAFETY), mx(ST_QP), mx(ST_TOTAL),
                (double)percentile_us(s, 50), (double)percentile_us(s, 99),
                (double)dt * 1e6, 100.0 * avg(ST_TOTAL) / (dt * 1e6), 100.0 * mx(ST_TOTAL) / (dt * 1e6),
                (double)s.qp_iter_sum / s.steps, s.qp_iter_max, s.fallbacks,
                (double)s.min_h, s.violations, (double)s.max_pen * 1000, (double)s.final_err);
}

static void run_all(bool with_logs, int repeats) {
  ScenarioCfg cfgs[3];
  cfgs[0].name = "nominal_only"; cfgs[0].cbf = false;
  cfgs[1].name = "ideal";
  cfgs[2].name = "disturbed";    cfgs[2].disturbance = true;
  const real tpu = (real)getCpuFrequencyMhz();

  for (auto& c : cfgs) {
    c.log_every = with_logs ? 10 : 0;
    for (int rep = 0; rep < repeats; ++rep) {
      BenchStats s;
      s.hist = hist;
      if (with_logs) {
        Serial.printf("#BEGIN %s\n", c.name);
        Serial.println("t,q1,q2,q3,u1,u2,u3,dh1,dh2,dh3,d1,d2,d3,h_w1,h_w2,h_min,qp_iters,fallback,pen_mm,ctrl_us,x2,y2,z2");
      }
      run_scenario(c, clock_cycles, tpu, with_logs ? log_row : nullptr, nullptr, s);
      print_summary(c.name, s, c.dt);
      if (with_logs) Serial.printf("#END %s\n", c.name);
    }
  }
  Serial.printf("#INFO stack_free_bytes=%u heap_free_bytes=%u\n",
                (unsigned)uxTaskGetStackHighWaterMark(nullptr), (unsigned)ESP.getFreeHeap());
  Serial.println("#DONE");
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) delay(10);  // native USB: wait for the host
  delay(500);
  Serial.printf("#INFO board=YoloUNO chip=%s rev=%d cores=%d cpu_mhz=%u scalar=%s flash=%u\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned)getCpuFrequencyMhz(), sizeof(real) == 4 ? "float" : "double",
                (unsigned)ESP.getFlashChipSize());
  run_all(true, 1);
  Serial.println("#INFO send 'r' to rerun with logs, 'b' for benchmark-only runs");
}

void loop() {
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'r') run_all(true, 1);
    else if (c == 'b') run_all(false, 5);
  }
  delay(10);
}
