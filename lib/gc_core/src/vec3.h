// vec3.h - tiny 3-vector templated on scalar (real or Jet).
#pragma once
#include <cmath>
#include "jet.h"

namespace gc {

template <class S> struct V3 { S x, y, z; };
using V3r = V3<real>;

template <class S> inline V3<S> operator+(const V3<S>& a, const V3<S>& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
template <class S> inline V3<S> operator-(const V3<S>& a, const V3<S>& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
template <class S> inline V3<S> scale(const S& k, const V3<S>& a) { return {k * a.x, k * a.y, k * a.z}; }
// Dot product of a constant (real) vector with a templated vector.
template <class S> inline S dotn(const V3r& n, const V3<S>& p) { return n.x * p.x + n.y * p.y + n.z * p.z; }

inline real dot(const V3r& a, const V3r& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3r cross(const V3r& a, const V3r& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline V3r normalized(const V3r& a) { real n = std::sqrt(dot(a, a)); return {a.x / n, a.y / n, a.z / n}; }

}  // namespace gc
