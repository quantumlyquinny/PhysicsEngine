#pragma once
#include "SupportPoint.hpp"
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// GJK Simplex — holds 1 to 4 support points.
//
// Naming convention (consistent throughout GJK):
//   points[0] = A = the NEWEST point (just added this iteration)
//   points[1] = B
//   points[2] = C
//   points[3] = D = oldest (only present in the tetrahedron case)
//
// The simplex case functions (line/triangle/tetrahedron) are written assuming
// this layout. Violating the convention will produce wrong search directions.
// ─────────────────────────────────────────────────────────────────────────────
struct Simplex {
    SupportPoint points[4];
    int          size = 0;

    // Push a new point to the front, making it A (newest).
    // Older points shift back by one index; point at index 3 is discarded
    // if size was already 4 (should not happen in a correct GJK run).
    void push_front(const SupportPoint& p) {
        points[3] = points[2];
        points[2] = points[1];
        points[1] = points[0];
        points[0] = p;
        size = std::min(size + 1, 4);
    }

    // Explicit setters — used by case handlers to reduce the simplex.
    // Always copy points BEFORE calling these if the source IS this simplex.
    void set(const SupportPoint& a) {
        points[0] = a; size = 1;
    }
    void set(const SupportPoint& a, const SupportPoint& b) {
        points[0] = a; points[1] = b; size = 2;
    }
    void set(const SupportPoint& a, const SupportPoint& b,
             const SupportPoint& c) {
        points[0] = a; points[1] = b; points[2] = c; size = 3;
    }
    void set(const SupportPoint& a, const SupportPoint& b,
             const SupportPoint& c, const SupportPoint& d) {
        points[0] = a; points[1] = b; points[2] = c; points[3] = d; size = 4;
    }

    const SupportPoint& operator[](int i) const { return points[i]; }
          SupportPoint& operator[](int i)       { return points[i]; }
};