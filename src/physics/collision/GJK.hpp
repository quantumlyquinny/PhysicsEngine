#pragma once
#include "Simplex.hpp"
#include "SupportPoint.hpp"

struct GJKResult {
    bool    collision = false;
    Simplex simplex;  // If collision == true, this is the tetrahedron enclosing origin.
                      // EPA consumes this directly — do not discard.
};

class GJK {
public:
    // Maximum iterations before declaring convergence.
    // 64 is conservative; typical convergence is 6–15 iterations.
    static constexpr int MAX_ITERATIONS = 64;

    // Run GJK between two world-transformed convex shapes.
    // Returns a collision flag and the final simplex.
    static GJKResult test(const TransformedShape& shapeA,
                          const TransformedShape& shapeB);

private:
    // Compute the support point on the Minkowski difference A ⊖ B.
    static SupportPoint support(const TransformedShape& A,
                                const TransformedShape& B,
                                const glm::vec3& dir);

    // The core simplex evolution function.
    // Mutates simplex to the minimal feature closest to origin.
    // Mutates dir to point from that feature toward origin.
    // Returns true if the origin is proven to be inside the simplex.
    static bool doSimplex(Simplex& simplex, glm::vec3& dir);

    // ── Per-dimension case handlers ────────────────────────────────────────
    // Each receives the current simplex and direction by reference.
    // Mutates both. Returns true only if origin is provably enclosed.
    static bool lineCase       (Simplex& simplex, glm::vec3& dir);
    static bool triangleCase   (Simplex& simplex, glm::vec3& dir);
    static bool tetrahedronCase(Simplex& simplex, glm::vec3& dir);
};