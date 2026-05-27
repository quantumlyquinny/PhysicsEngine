#pragma once
#include <glm/glm.hpp>
#include "../RigidBody.hpp" // for BodyID

// ─────────────────────────────────────────────────────────────────────────────
// Manifold — the complete contact description produced by EPA.
//
// The contact solver (Step 5) reads every field of this struct.
// Populate nothing by hand — always produced by EPA::solve().
// ─────────────────────────────────────────────────────────────────────────────
struct Manifold {
    BodyID    bodyA            = INVALID_BODY_ID;
    BodyID    bodyB            = INVALID_BODY_ID;

    // Unit normal pointing FROM body B's surface INTO body A.
    // Impulses applied along this axis separate the two bodies.
    glm::vec3 normal           = glm::vec3(0.0f);

    // Signed penetration depth (always >= 0 for a valid manifold).
    // Depth > 0 means the shapes are overlapping.
    float     depth            = 0.0f;

    // World-space contact point on the surface of body A.
    glm::vec3 contactPointA    = glm::vec3(0.0f);

    // World-space contact point on the surface of body B.
    glm::vec3 contactPointB    = glm::vec3(0.0f);

    // True if EPA converged to a valid contact — always check before using.
    bool      isValid          = false;
};