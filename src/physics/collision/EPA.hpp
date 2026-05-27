#pragma once
#include "Simplex.hpp"
#include "SupportPoint.hpp"
#include "Manifold.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// EPA — Expanding Polytope Algorithm.
//
// Starts from the GJK tetrahedron (which encloses the origin) and expands it
// by repeatedly finding the face closest to the origin, adding the support
// point in that face's direction, and rebuilding the polytope.
//
// Converges when the new support point is within TOLERANCE of the closest face.
// Output: penetration depth, contact normal, and world-space contact points.
//
// All storage is stack-allocated — zero heap allocation per call.
// ─────────────────────────────────────────────────────────────────────────────
class EPA {
public:
    static constexpr int   MAX_FACES      = 64;
    static constexpr int   MAX_EDGES      = 64;
    static constexpr int   MAX_ITERATIONS = 64;
    static constexpr float TOLERANCE      = 1e-4f;

    static Manifold solve(const Simplex&         gjkSimplex,
                          const TransformedShape& shapeA,
                          const TransformedShape& shapeB);

private:
    // ── EPA Face ──────────────────────────────────────────────────────────
    // Vertices stored in CCW order as seen from outside the polytope.
    // normal always points AWAY from the origin (distance >= 0).
    struct Face {
        SupportPoint verts[3];
        glm::vec3    normal;   // unit outward normal
        float        distance; // signed distance from origin to face plane (>= 0)
    };

    // ── EPA Edge ─────────────────────────────────────────────────────────
    // Silhouette edge: boundary between faces visible and not visible from
    // the new support point. New faces are stitched along these edges.
    struct Edge {
        SupportPoint a, b; // winding: a→b as seen from outside the polytope
    };

    // ── Helpers ───────────────────────────────────────────────────────────

    // Construct a face from three support points.
    // Automatically orients the normal to point away from the origin.
    static Face makeFace(const SupportPoint& a,
                         const SupportPoint& b,
                         const SupportPoint& c);

    // Return the index of the face with the smallest distance from the origin.
    static int findClosestFace(const Face* faces, int count);

    // Compute barycentric coordinates of the origin projected onto triangle ABC.
    // Returns (u, v, w) where u is the weight for vertex a.
    static glm::vec3 barycentricOrigin(const glm::vec3& a,
                                       const glm::vec3& b,
                                       const glm::vec3& c);
};