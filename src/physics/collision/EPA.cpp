#include "EPA.hpp"
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
EPA::Face EPA::makeFace(const SupportPoint& a,
                        const SupportPoint& b,
                        const SupportPoint& c)
{
    Face face;
    face.verts[0] = a;
    face.verts[1] = b;
    face.verts[2] = c;

    const glm::vec3 ab = b.minkowski - a.minkowski;
    const glm::vec3 ac = c.minkowski - a.minkowski;
    const glm::vec3 rawNormal = glm::cross(ab, ac);

    const float rawLen = glm::length(rawNormal);

    // Degenerate face (zero area) — assign a safe fallback
    if (rawLen < 1e-10f) {
        face.normal   = glm::vec3(0.0f, 1.0f, 0.0f);
        face.distance = glm::dot(face.normal, a.minkowski);
        if (face.distance < 0.0f) {
            face.normal   = -face.normal;
            face.distance = -face.distance;
        }
        return face;
    }

    face.normal   = rawNormal / rawLen; // normalise
    face.distance = glm::dot(face.normal, a.minkowski);

    // Ensure normal points AWAY from origin (positive distance convention)
    if (face.distance < 0.0f) {
        face.normal   = -face.normal;
        face.distance = -face.distance;
        // Swap B and C to maintain consistent CCW winding with the flipped normal
        std::swap(face.verts[1], face.verts[2]);
    }

    return face;
}

// ─────────────────────────────────────────────────────────────────────────────
int EPA::findClosestFace(const Face* faces, int count) {
    int   best     = 0;
    float minDist  = faces[0].distance;
    for (int i = 1; i < count; i++) {
        if (faces[i].distance < minDist) {
            minDist = faces[i].distance;
            best    = i;
        }
    }
    return best;
}

// ─────────────────────────────────────────────────────────────────────────────
glm::vec3 EPA::barycentricOrigin(const glm::vec3& a,
                                 const glm::vec3& b,
                                 const glm::vec3& c)
{
    // Project origin (0,0,0) onto the plane of triangle ABC.
    // Solve: origin_proj = a + s*(b-a) + t*(c-a)
    //   → ap = s*ab + t*ac   where ap = proj - a
    //
    // Using the dot-product system (Cramer's rule):
    //   dot(ab,ab)*s + dot(ab,ac)*t = dot(ap,ab)
    //   dot(ab,ac)*s + dot(ac,ac)*t = dot(ap,ac)

    const glm::vec3 ab = b - a;
    const glm::vec3 ac = c - a;

    // Compute the normal and project origin onto the plane
    const glm::vec3 n       = glm::cross(ab, ac);
    const float     nLenSq  = glm::dot(n, n);
    if (nLenSq < 1e-14f) {
        // Degenerate — return centroid weights
        return glm::vec3(1.0f / 3.0f);
    }

    // Origin projected onto the plane
    const float     d  = glm::dot(n, a) / std::sqrt(nLenSq);
    const glm::vec3 p  = (glm::dot(n, a) / nLenSq) * n; // origin's projection

    // ap = p - a
    const glm::vec3 ap = p - a;

    const float d1 = glm::dot(ab, ab);
    const float d2 = glm::dot(ab, ac);
    const float d3 = glm::dot(ac, ac);
    const float e1 = glm::dot(ap, ab);
    const float e2 = glm::dot(ap, ac);

    const float denom = d1 * d3 - d2 * d2;
    if (std::abs(denom) < 1e-14f) {
        return glm::vec3(1.0f / 3.0f);
    }

    const float s = (e1 * d3 - e2 * d2) / denom; // weight for b
    const float t = (e2 * d1 - e1 * d2) / denom; // weight for c
    const float u = 1.0f - s - t;                // weight for a

    // Clamp and renormalise to guard against floating-point drift
    const float cu = std::max(0.0f, u);
    const float cs = std::max(0.0f, s);
    const float ct = std::max(0.0f, t);
    const float sum = cu + cs + ct;
    if (sum < 1e-10f) return glm::vec3(1.0f / 3.0f);

    return glm::vec3(cu, cs, ct) / sum;
}

// ─────────────────────────────────────────────────────────────────────────────
Manifold EPA::solve(const Simplex&         gjkSimplex,
                    const TransformedShape& shapeA,
                    const TransformedShape& shapeB)
{
    Manifold result;

    // ── Require a tetrahedron from GJK ────────────────────────────────────
    // EPA starts from a polytope that ENCLOSES the origin.
    // Only the tetrahedron case guarantees this. Degenerate simplices
    // (line, triangle) happen on exact touching — report a zero manifold.
    if (gjkSimplex.size != 4) {
        result.isValid = false;
        return result;
    }

    // ── Initialise polytope from the GJK tetrahedron ABCD ─────────────────
    // Stack-allocated — zero heap activity.
    Face faces[MAX_FACES];
    int  faceCount = 0;

    const auto addFace = [&](const SupportPoint& a,
                              const SupportPoint& b,
                              const SupportPoint& c)
    {
        if (faceCount < MAX_FACES) {
            faces[faceCount++] = makeFace(a, b, c);
        }
    };

    const SupportPoint& A = gjkSimplex[0];
    const SupportPoint& B = gjkSimplex[1];
    const SupportPoint& C = gjkSimplex[2];
    const SupportPoint& D = gjkSimplex[3];

    // Four faces of the tetrahedron.
    // makeFace() orients each normal to point away from the origin automatically.
    addFace(A, B, C);  // face opposite D
    addFace(A, C, D);  // face opposite B
    addFace(A, D, B);  // face opposite C
    addFace(B, D, C);  // face opposite A  (note: BDC — winding is deliberate)

    // ── EPA iteration ─────────────────────────────────────────────────────
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // ── Find the face closest to the origin ───────────────────────────
        const int   closestIdx  = findClosestFace(faces, faceCount);
        const Face& closestFace = faces[closestIdx];
        const glm::vec3& n      = closestFace.normal;

        // ── New support point in the direction of the closest face ─────────
        const SupportPoint newSP(shapeA.support(n), shapeB.support(-n));
        const float        newDist = glm::dot(newSP.minkowski, n);

        // ── Convergence test ──────────────────────────────────────────────
        // If the new support point is not significantly further from origin
        // than the current closest face, we've found the minimum penetration.
        if (newDist - closestFace.distance < TOLERANCE) {
            // ── Extract contact from the closest face ─────────────────────
            // Barycentric coordinates of origin projected onto the face
            const glm::vec3 bary = barycentricOrigin(
                closestFace.verts[0].minkowski,
                closestFace.verts[1].minkowski,
                closestFace.verts[2].minkowski
            );

            // Interpolate world-space contact points using barycentric weights.
            // bary.x → weight for verts[0], bary.y → verts[1], bary.z → verts[2]
            result.contactPointA =
                bary.x * closestFace.verts[0].pointA +
                bary.y * closestFace.verts[1].pointA +
                bary.z * closestFace.verts[2].pointA;

            result.contactPointB =
                bary.x * closestFace.verts[0].pointB +
                bary.y * closestFace.verts[1].pointB +
                bary.z * closestFace.verts[2].pointB;

            result.normal  = n;
            result.depth   = closestFace.distance;
            result.isValid = true;
            return result;
        }

        // ── Expand the polytope ───────────────────────────────────────────
        // Find all faces visible from the new support point (those whose
        // outward normal has a positive component toward newSP).
        // Collect their silhouette edges — the boundary of the visible region.

        Edge silhouette[MAX_EDGES];
        int  edgeCount = 0;

        // Iterate backward so removing by swap-with-last is safe
        int i = 0;
        while (i < faceCount) {
            const Face& f      = faces[i];
            const glm::vec3 toNew = newSP.minkowski - f.verts[0].minkowski;

            if (glm::dot(f.normal, toNew) > 0.0f) {
                // Face is visible — process its three edges
                for (int e = 0; e < 3; e++) {
                    const SupportPoint& ea = f.verts[e];
                    const SupportPoint& eb = f.verts[(e + 1) % 3];

                    // Check if the REVERSE of this edge already exists
                    // in the silhouette list. Reverse edge = shared with another
                    // visible face = interior edge → cancel both out.
                    bool cancelled = false;
                    for (int k = 0; k < edgeCount; k++) {
                        constexpr float EPS_SQ = 1e-10f;
                        const bool aMatch = glm::dot(
                            silhouette[k].a.minkowski - eb.minkowski,
                            silhouette[k].a.minkowski - eb.minkowski) < EPS_SQ;
                        const bool bMatch = glm::dot(
                            silhouette[k].b.minkowski - ea.minkowski,
                            silhouette[k].b.minkowski - ea.minkowski) < EPS_SQ;

                        if (aMatch && bMatch) {
                            // Remove by swap-with-last — O(1)
                            silhouette[k] = silhouette[--edgeCount];
                            cancelled = true;
                            break;
                        }
                    }

                    if (!cancelled && edgeCount < MAX_EDGES) {
                        silhouette[edgeCount++] = { ea, eb };
                    }
                }

                // Remove this visible face: swap with last, decrement count
                faces[i] = faces[--faceCount];
                // Do NOT increment i — re-check the face now at position i
            } else {
                i++;
            }
        }

        // ── Stitch new faces along each silhouette edge ───────────────────
        // Each new face is: (newSP, silhouette[e].a, silhouette[e].b)
        // makeFace() handles orientation automatically.
        for (int e = 0; e < edgeCount; e++) {
            addFace(newSP, silhouette[e].a, silhouette[e].b);
        }
    }

    // ── Max iterations: return the best approximation so far ──────────────
    const int   closestIdx  = findClosestFace(faces, faceCount);
    const Face& closestFace = faces[closestIdx];

    const glm::vec3 bary = barycentricOrigin(
        closestFace.verts[0].minkowski,
        closestFace.verts[1].minkowski,
        closestFace.verts[2].minkowski
    );

    result.contactPointA =
        bary.x * closestFace.verts[0].pointA +
        bary.y * closestFace.verts[1].pointA +
        bary.z * closestFace.verts[2].pointA;

    result.contactPointB =
        bary.x * closestFace.verts[0].pointB +
        bary.y * closestFace.verts[1].pointB +
        bary.z * closestFace.verts[2].pointB;

    result.normal  = closestFace.normal;
    result.depth   = closestFace.distance;
    result.isValid = true;
    return result;
}