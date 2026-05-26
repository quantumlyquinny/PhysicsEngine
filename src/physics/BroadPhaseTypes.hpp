#pragma once
#include "../spatial/AABB.hpp"
#include "RigidBody.hpp"

// ── Data fed into the Octree each step ───────────────────────────────────────
// Pre-computed by BroadPhase from RigidBody state. Octree is deliberately
// decoupled from RigidBody — it operates only on IDs and AABBs.
struct BodyEntry {
    BodyID id;
    AABB   aabb;
    bool   isStatic;
    bool   isSleeping;
};

// ── Output of the broad phase ─────────────────────────────────────────────────
// Invariant: a < b. Enforced at insertion time to prevent (A,B)+(B,A) duplicates.
struct BodyPair {
    BodyID a, b;

    bool operator==(const BodyPair& o) const { return a == o.a && b == o.b; }
    bool operator<(const BodyPair& o)  const {
        return a < o.a || (a == o.a && b < o.b);
    }
};