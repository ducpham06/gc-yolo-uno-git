// jet.h - forward-mode AD number carrying everything the HOCBF needs in one pass:
//   v = h(q),  g = dh/dq (Jacobian J_h),  d = J_h*qdot (= hdot),  s = qdot^T Hess(h) qdot (= Jdot*qdot)
#pragma once
#include <cmath>
#include "gc_config.h"

namespace gc {

struct Jet {
  real v, d, s;
  real g[NJ];
  Jet() : v(0), d(0), s(0) { for (int i = 0; i < NJ; ++i) g[i] = 0; }
  Jet(real c) : v(c), d(0), s(0) { for (int i = 0; i < NJ; ++i) g[i] = 0; }  // NOLINT (implicit on purpose)
};

inline Jet operator+(const Jet& a, const Jet& b) {
  Jet r; r.v = a.v + b.v; r.d = a.d + b.d; r.s = a.s + b.s;
  for (int i = 0; i < NJ; ++i) r.g[i] = a.g[i] + b.g[i];
  return r;
}
inline Jet operator-(const Jet& a, const Jet& b) {
  Jet r; r.v = a.v - b.v; r.d = a.d - b.d; r.s = a.s - b.s;
  for (int i = 0; i < NJ; ++i) r.g[i] = a.g[i] - b.g[i];
  return r;
}
inline Jet operator-(const Jet& a) {
  Jet r; r.v = -a.v; r.d = -a.d; r.s = -a.s;
  for (int i = 0; i < NJ; ++i) r.g[i] = -a.g[i];
  return r;
}
inline Jet operator*(const Jet& a, const Jet& b) {
  Jet r; r.v = a.v * b.v; r.d = a.d * b.v + a.v * b.d;
  r.s = a.s * b.v + real(2) * a.d * b.d + a.v * b.s;
  for (int i = 0; i < NJ; ++i) r.g[i] = a.g[i] * b.v + a.v * b.g[i];
  return r;
}
// Scalar function f with f(x.v)=f0, f'=f1, f''=f2 applied to a jet.
inline Jet apply(const Jet& x, real f0, real f1, real f2) {
  Jet r; r.v = f0; r.d = f1 * x.d; r.s = f1 * x.s + f2 * x.d * x.d;
  for (int i = 0; i < NJ; ++i) r.g[i] = f1 * x.g[i];
  return r;
}
inline Jet operator/(const Jet& a, const Jet& b) {
  real iv = real(1) / b.v;
  return a * apply(b, iv, -iv * iv, real(2) * iv * iv * iv);
}

// Overloads so the same template code runs on plain reals and on jets.
inline real val(real x) { return x; }
inline real val(const Jet& x) { return x.v; }
inline real m_sin(real x) { return std::sin(x); }
inline real m_cos(real x) { return std::cos(x); }
inline real m_tanh(real x) { return std::tanh(x); }
inline Jet m_sin(const Jet& x) { real s = std::sin(x.v), c = std::cos(x.v); return apply(x, s, c, -s); }
inline Jet m_cos(const Jet& x) { real s = std::sin(x.v), c = std::cos(x.v); return apply(x, c, -s, -c); }
inline Jet m_tanh(const Jet& x) {
  real t = std::tanh(x.v), d1 = real(1) - t * t;
  return apply(x, t, d1, real(-2) * t * d1);
}

}  // namespace gc
