// qp.h - min 0.5||u - u_nom||^2  s.t.  A u <= b   (Eq. 19)
// Solved with Hildreth's dual coordinate-ascent method: no matrix factorisation,
// O(m n) per sweep, warm-startable, fixed memory -> suitable for an MCU.
#pragma once
#include "gc_config.h"

namespace gc {

struct QPResult {
  int iters;       // sweeps used
  real max_viol;   // max_i (A_i u - b_i) after solve (rows unit-normalised)
  bool ok;         // converged and feasible within tolerance
};

// Rows of A must be unit-normalised. lambda is in/out (warm start).
QPResult qp_hildreth(const real A[][NJ], const real b[], int m, const real unom[NJ],
                     real lambda[], real u[NJ], int max_iter, real tol);

}  // namespace gc
