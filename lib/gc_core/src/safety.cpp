#include "safety.h"

namespace gc {

bool add_face(Workspace& W, const V3r* v, int nv, const MetricParams& mp) {
  if (W.nf >= MAX_FACES || nv < 3 || nv > MAX_VERTS) return false;
  Face& F = W.f[W.nf];
  F.n = normalized(cross(v[1] - v[0], v[2] - v[0]));
  F.c = dot(F.n, v[0]);
  V3r O = {0, 0, 0};
  for (int j = 0; j < nv; ++j) O = O + v[j];
  F.O = scale(real(1) / real(nv), O);
  F.ne = nv;
  for (int j = 0; j < nv; ++j) {
    const V3r a = v[j], b = v[(j + 1) % nv];
    F.en[j] = normalized(cross(F.n, b - a));   // n_kj ⟂ (b-a), n_kj ⟂ n_k
    F.ed[j] = dot(F.en[j], a);
    F.psiO[j] = std::tanh(mp.alpha * (dot(F.en[j], F.O) - F.ed[j]));
  }
  F.c2 = real(nv) - mp.c2_offset;
  ++W.nf;
  return true;
}

void build_wedge_workspace(Workspace& W, const MetricParams& mp) {
  // Same geometry as the MATLAB simulation: box [-0.5, 2.8] x [-2, 2] x [-1, 2] with the
  // wedge {x >= XA + |y|} removed. The wedge planes x -/+ y = 0.8 reach the far corners
  // (2.8, +-2), so the x = 2.8 wall disappears entirely.
  const real X0 = -0.5f, X1 = 2.8f, Y0 = -2, Y1 = 2, Z0 = -1, Z1 = 2;
  const real XA = 0.8f;
  W.nf = 0;

  // Floor and ceiling: rectangle minus triangle -> 3 convex pieces each (Assumption 2).
  for (int s = 0; s < 2; ++s) {
    const real z = s ? Z1 : Z0;
    V3r a[4] = {{X0, Y0, z}, {XA, Y0, z}, {XA, Y1, z}, {X0, Y1, z}};
    V3r b[3] = {{XA, 0, z}, {X1, Y1, z}, {XA, Y1, z}};
    V3r c[3] = {{XA, 0, z}, {XA, Y0, z}, {X1, Y0, z}};
    add_face(W, a, 4, mp); add_face(W, b, 3, mp); add_face(W, c, 3, mp);
  }
  // Side walls.
  { V3r v[4] = {{X0, Y0, Z0}, {X0, Y1, Z0}, {X0, Y1, Z1}, {X0, Y0, Z1}}; add_face(W, v, 4, mp); }
  { V3r v[4] = {{X0, Y1, Z0}, {X1, Y1, Z0}, {X1, Y1, Z1}, {X0, Y1, Z1}}; add_face(W, v, 4, mp); }
  { V3r v[4] = {{X0, Y0, Z0}, {X1, Y0, Z0}, {X1, Y0, Z1}, {X0, Y0, Z1}}; add_face(W, v, 4, mp); }
  // Wedge faces: plane 1 (y >= 0, red) and plane 2 (y <= 0, magenta). Kept last.
  { V3r v[4] = {{XA, 0, Z0}, {X1, Y1, Z0}, {X1, Y1, Z1}, {XA, 0, Z1}}; add_face(W, v, 4, mp); }
  { V3r v[4] = {{XA, 0, Z0}, {X1, Y0, Z0}, {X1, Y0, Z1}, {XA, 0, Z1}}; add_face(W, v, 4, mp); }
}

}  // namespace gc
