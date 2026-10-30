#include "qp.h"
#include <cmath>

namespace gc {

QPResult qp_hildreth(const real A[][NJ], const real b[], int m, const real unom[NJ],
                     real lambda[], real u[NJ], int max_iter, real tol) {
  for (int k = 0; k < NJ; ++k) u[k] = unom[k];
  for (int i = 0; i < m; ++i)
    if (lambda[i] != 0)
      for (int k = 0; k < NJ; ++k) u[k] -= lambda[i] * A[i][k];

  QPResult res{0, 0, false};
  for (int it = 1; it <= max_iter; ++it) {
    real max_dl = 0;
    for (int i = 0; i < m; ++i) {
      real r = -b[i];
      for (int k = 0; k < NJ; ++k) r += A[i][k] * u[k];
      real ln = lambda[i] + r;          // exact maximiser along lambda_i (||A_i|| = 1)
      if (ln < 0) ln = 0;
      const real dl = ln - lambda[i];
      if (dl != 0) {
        for (int k = 0; k < NJ; ++k) u[k] -= dl * A[i][k];
        lambda[i] = ln;
        const real a = std::fabs(dl);
        if (a > max_dl) max_dl = a;
      }
    }
    res.iters = it;
    if (max_dl < tol) break;
  }
  real mv = 0;
  for (int i = 0; i < m; ++i) {
    real r = -b[i];
    for (int k = 0; k < NJ; ++k) r += A[i][k] * u[k];
    if (r > mv) mv = r;
  }
  res.max_viol = mv;
  res.ok = (mv <= real(10) * tol) && std::isfinite(u[0]) && std::isfinite(u[1]) && std::isfinite(u[2]);
  return res;
}

}  // namespace gc
