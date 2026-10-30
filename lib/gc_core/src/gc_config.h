// gc_config.h - compile-time configuration for the geometric-containment controller core.
#pragma once
#include <stdint.h>

// Scalar type. ESP32-S3 has a single-precision FPU only, so `double` is
// software-emulated there. Build both to compare (see platformio.ini).
#ifndef GC_REAL
#define GC_REAL float
#endif

namespace gc {
using real = GC_REAL;

constexpr int NJ = 3;          // joints (n-DOF)
constexpr int NL = 2;          // links modelled as segments (N)
constexpr int MAX_FACES = 16;  // convex faces of the workspace boundary (K)
constexpr int MAX_VERTS = 6;   // vertices (= edges) per convex face (J_k)
constexpr int MAX_CON = NL * MAX_FACES + 4 * NJ;  // QP rows: HOCBF + velocity CBF + torque box

enum Stage { ST_DYN = 0, ST_DOB_NOM, ST_SAFETY, ST_QP, ST_TOTAL, NSTAGE };
}  // namespace gc
