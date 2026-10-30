// safety.h - geometric containment metric h_ik = c + h1_ik + h2_ik (Eqs. 7-10).
#pragma once
#include "robot.h"

namespace gc {

struct MetricParams {
  real alpha = 10.0f;     // psi(z) = tanh(alpha z); 1/alpha ~ "look-ahead" distance [m]
  real c = 1.0f;          // 0 < c < 2 (Eq. 10). c=1 gives the steady value 3 seen in Fig. 5
  real c2_offset = 1.5f;  // c2_ik = J_k - c2_offset, must lie in (J_k-2, J_k) (Eq. 9)
  real par_eps = 1e-6f;   // |n^T (p_i - p_{i-1})| below this -> Remark 3 (I_ik := p_i)
};

// One convex planar face F_k with everything that is constant precomputed.
struct Face {
  V3r n;  real c;                 // plane H_k: n^T x = c
  V3r O;                          // reference point O_k in F_k (centroid)
  int ne;                         // J_k
  V3r en[MAX_VERTS];              // edge normals n_kj (in-plane, perpendicular to edge)
  real ed[MAX_VERTS];             // edge offsets d_kj
  real psiO[MAX_VERTS];           // psi(d(a_kj b_kj, O_k)) - constant, so cached
  real c2;                        // c2_ik
};

struct Workspace {
  Face f[MAX_FACES];
  int nf = 0;
};

// Build a face from an ordered list of vertices of a convex planar polygon.
bool add_face(Workspace& W, const V3r* v, int nv, const MetricParams& mp);

// Paper's Sec. IV scene: box with a vertical wedge-shaped cutout (non-convex P).
// Non-convex box faces are split into convex pieces (Assumption 2).
void build_wedge_workspace(Workspace& W, const MetricParams& mp);

template <class S> inline S psi(const S& z, real alpha) { return m_tanh(alpha * z); }

// h_ik for link segment [pa, pb] = [p_{i-1}, p_i] against face F.
template <class S>
S pair_metric(const V3<S>& pa, const V3<S>& pb, const Face& F, const MetricParams& mp) {
  const S sa = dotn(F.n, pa) - F.c;
  const S sb = dotn(F.n, pb) - F.c;
  const S h1 = psi(sa, mp.alpha) * psi(sb, mp.alpha);            // Eq. (7)

  const V3<S> e = pb - pa;
  const S den = dotn(F.n, e);
  V3<S> I;
  if (std::fabs(val(den)) < mp.par_eps) {
    I = pb;                                                     // Remark 3
  } else {
    I = pb - scale(S(sb / den), e);                             // Eq. (8)
  }

  S sum(0);
  for (int j = 0; j < F.ne; ++j) {
    const S dI = dotn(F.en[j], I) - F.ed[j];
    sum = sum + psi(dI, mp.alpha) * F.psiO[j];
  }
  const S h2 = psi(S(F.c2) - sum, mp.alpha);                    // Eq. (9)
  return S(mp.c) + h1 + h2;                                     // Eq. (10)
}

}  // namespace gc
