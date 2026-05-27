#include "GJK.hpp"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
SupportPoint GJK::support(const TransformedShape& A,
                           const TransformedShape& B,
                           const glm::vec3& dir)
{
    // Support on the Minkowski difference:
    //   furthest point on A in  +dir
    //   furthest point on B in  -dir
    //   their difference is the support on A ⊖ B
    return SupportPoint(A.support(dir), B.support(-dir));
}

// ─────────────────────────────────────────────────────────────────────────────
// LINE CASE
// Simplex: [A, B]   A = newest (index 0)
//
// Two Voronoi regions to test:
//   Region AB : origin is between A and B along edge AB
//   Region A  : origin is past A, away from B
//
// We never test "region B" because GJK only adds A when A is PAST the current
// simplex in direction of the origin — B is already known to be closer than A
// in the search direction.
// ─────────────────────────────────────────────────────────────────────────────
bool GJK::lineCase(Simplex& simplex, glm::vec3& dir) {
    const SupportPoint A = simplex[0]; // copy — we may overwrite simplex
    const SupportPoint B = simplex[1];

    const glm::vec3 AB = B.minkowski - A.minkowski;
    const glm::vec3 AO = -A.minkowski; // vector from A to origin

    if (glm::dot(AB, AO) > 0.0f) {
        // Origin lies between A and B — keep both, new direction perpendicular to AB
        // Triple product (AB × AO) × AB gives the component of AO perpendicular to AB
        dir = glm::cross(glm::cross(AB, AO), AB);
        simplex.set(A, B);
    } else {
        // Origin is past A in the -AB direction — discard B, reduce to point
        dir = AO;
        simplex.set(A);
    }

    return false; // A line can never enclose the origin in 3D
}

// ─────────────────────────────────────────────────────────────────────────────
// TRIANGLE CASE
// Simplex: [A, B, C]   A = newest (index 0)
//
// We only test edges that contain A, because B and C were already the closest
// feature before A was added. The origin must be "on the A side".
//
// Voronoi regions:
//   Edge AB   : cross(AB, ABC) gives the outward normal to AB (away from C)
//   Edge AC   : cross(ABC, AC) gives the outward normal to AC (away from B)
//   Above face: origin is on the +ABC normal side of the triangle
//   Below face: origin is on the -ABC normal side of the triangle
// ─────────────────────────────────────────────────────────────────────────────
bool GJK::triangleCase(Simplex& simplex, glm::vec3& dir) {
    const SupportPoint A = simplex[0]; // copy before mutating simplex
    const SupportPoint B = simplex[1];
    const SupportPoint C = simplex[2];

    const glm::vec3 AB  = B.minkowski - A.minkowski;
    const glm::vec3 AC  = C.minkowski - A.minkowski;
    const glm::vec3 AO  = -A.minkowski;
    const glm::vec3 ABC = glm::cross(AB, AC); // face normal (winding: A→B→C)

    // ── Test edge AB ──────────────────────────────────────────────────────
    // cross(AB, ABC) is perpendicular to AB, pointing away from C (outward)
    if (glm::dot(glm::cross(AB, ABC), AO) > 0.0f) {
        if (glm::dot(AB, AO) > 0.0f) {
            // Origin is in the Voronoi region of edge AB
            simplex.set(A, B);
            dir = glm::cross(glm::cross(AB, AO), AB);
        } else {
            // Origin is past A only — degenerate; reduce to point
            simplex.set(A);
            dir = AO;
        }
        return false;
    }

    // ── Test edge AC ──────────────────────────────────────────────────────
    // cross(ABC, AC) is perpendicular to AC, pointing away from B (outward)
    if (glm::dot(glm::cross(ABC, AC), AO) > 0.0f) {
        if (glm::dot(AC, AO) > 0.0f) {
            // Origin is in the Voronoi region of edge AC
            simplex.set(A, C);
            dir = glm::cross(glm::cross(AC, AO), AC);
        } else {
            simplex.set(A);
            dir = AO;
        }
        return false;
    }

    // ── Origin is within the triangular prism — test above vs. below ──────
    if (glm::dot(ABC, AO) > 0.0f) {
        // Origin is above the triangle (on the +ABC normal side)
        // Keep [A, B, C], point direction toward origin along face normal
        simplex.set(A, B, C);
        dir = ABC;
    } else {
        // Origin is below — flip winding so normal points toward origin
        // Swapping B and C reverses the face normal: new normal = -ABC
        simplex.set(A, C, B);
        dir = -ABC;
    }

    return false; // A triangle cannot enclose the origin in 3D
}

// ─────────────────────────────────────────────────────────────────────────────
// TETRAHEDRON CASE
// Simplex: [A, B, C, D]   A = newest (index 0)
//
// Three faces to test (all contain A — face BCD is excluded because we know
// origin is already on the A-side from the previous triangle step):
//   Face ABC  (opposite D)
//   Face ACD  (opposite B)
//   Face ADB  (opposite C)
//
// For each face, we compute a normal that points OUTWARD (away from the 4th
// vertex) and test whether the origin is outside that face. If so, the problem
// reduces to the triangle case for that face.
//
// If origin is inside all three half-spaces → inside the tetrahedron → COLLISION.
// ─────────────────────────────────────────────────────────────────────────────
bool GJK::tetrahedronCase(Simplex& simplex, glm::vec3& dir) {
    const SupportPoint A = simplex[0]; // copy before any mutation
    const SupportPoint B = simplex[1];
    const SupportPoint C = simplex[2];
    const SupportPoint D = simplex[3];

    const glm::vec3 AB = B.minkowski - A.minkowski;
    const glm::vec3 AC = C.minkowski - A.minkowski;
    const glm::vec3 AD = D.minkowski - A.minkowski;
    const glm::vec3 AO = -A.minkowski;

    // ── Face normals via cross products ───────────────────────────────────
    glm::vec3 fABC = glm::cross(AB, AC);
    glm::vec3 fACD = glm::cross(AC, AD);
    glm::vec3 fADB = glm::cross(AD, AB);

    // Ensure each normal points OUTWARD (away from the opposite vertex).
    // If dot(fABC, AD) > 0, fABC points TOWARD D → flip it.
    if (glm::dot(fABC, AD) > 0.0f) fABC = -fABC;
    if (glm::dot(fACD, AB) > 0.0f) fACD = -fACD;
    if (glm::dot(fADB, AC) > 0.0f) fADB = -fADB;

    // ── Check each face ───────────────────────────────────────────────────
    // If origin is outside face ABC → reduce to that triangle
    if (glm::dot(fABC, AO) > 0.0f) {
        simplex.set(A, B, C);
        return triangleCase(simplex, dir);
    }
    // If origin is outside face ACD → reduce to that triangle
    if (glm::dot(fACD, AO) > 0.0f) {
        simplex.set(A, C, D);
        return triangleCase(simplex, dir);
    }
    // If origin is outside face ADB → reduce to that triangle
    if (glm::dot(fADB, AO) > 0.0f) {
        simplex.set(A, D, B);
        return triangleCase(simplex, dir);
    }

    // Origin is inside all three faces → enclosed by the tetrahedron → COLLISION
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool GJK::doSimplex(Simplex& simplex, glm::vec3& dir) {
    switch (simplex.size) {
        case 2: return lineCase       (simplex, dir);
        case 3: return triangleCase   (simplex, dir);
        case 4: return tetrahedronCase(simplex, dir);
        default: return false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
GJKResult GJK::test(const TransformedShape& shapeA,
                    const TransformedShape& shapeB)
{
    GJKResult result;

    // ── Choose an initial search direction ───────────────────────────────
    // The vector between shape centres is a good heuristic — it often
    // converges in far fewer iterations than an arbitrary direction.
    glm::vec3 dir = shapeA.position - shapeB.position;

    // Guard: shapes at the same position — use an arbitrary non-zero direction
    if (glm::dot(dir, dir) < 1e-8f) {
        dir = glm::vec3(1.0f, 0.0f, 0.0f);
    }

    // ── First support point ───────────────────────────────────────────────
    SupportPoint firstSP = support(shapeA, shapeB, dir);
    result.simplex.push_front(firstSP);

    // New search direction: from the first support point toward the origin
    dir = -firstSP.minkowski;

    // ── Main GJK loop ─────────────────────────────────────────────────────
    for (int i = 0; i < MAX_ITERATIONS; i++) {
        // Zero-length direction: shapes are touching exactly at a point.
        // Treat as a collision — EPA will report zero penetration depth.
        if (glm::dot(dir, dir) < 1e-10f) {
            result.collision = true;
            return result;
        }

        SupportPoint newSP = support(shapeA, shapeB, dir);

        // KEY TERMINATION TEST:
        // If the new point does not pass the origin in direction `dir`, the
        // origin cannot be inside the Minkowski difference — no collision.
        // dot(newSP.minkowski, dir) is the signed distance the new point
        // achieved in `dir`. If it's ≤ 0, we've reached the boundary.
        if (glm::dot(newSP.minkowski, dir) < 0.0f) {
            result.collision = false;
            return result;
        }

        result.simplex.push_front(newSP);

        // Evolve the simplex toward the origin.
        // doSimplex returns true only when we have a tetrahedron enclosing origin.
        if (doSimplex(result.simplex, dir)) {
            result.collision = true;
            return result;
        }
    }

    // Reached maximum iterations — conservative: treat as collision
    result.collision = true;
    return result;
}