// NarrowPhase.cpp
#include "NarrowPhase.hpp"

NarrowPhase::NarrowPhase(std::size_t maxExpectedPairs) {
    // Reserve generously — manifold count ≤ candidate pair count
    m_manifolds.reserve(maxExpectedPairs);
}

void NarrowPhase::detectCollisions(const std::vector<RigidBody>& bodies,
                                   const std::vector<BodyPair>&  pairs)
{
    m_manifolds.clear();
    m_gjkTests = m_gjkHits = m_epaFails = 0;

    for (const BodyPair& pair : pairs) {
        // ── Fetch bodies ──────────────────────────────────────────────────
        // Bodies are stored indexed by BodyID — direct access, no search.
        if (pair.a >= bodies.size() || pair.b >= bodies.size()) continue;
        const RigidBody& bodyA = bodies[pair.a];
        const RigidBody& bodyB = bodies[pair.b];
        if (bodyA.id == INVALID_BODY_ID || bodyB.id == INVALID_BODY_ID) continue;

        // ── Require collision shapes ──────────────────────────────────────
        if (!bodyA.collisionShape || !bodyB.collisionShape) continue;

        // ── Build world-space transformed shapes ──────────────────────────
        // Uses PHYSICS state (position/orientation), never render state.
        const TransformedShape tA = TransformedShape::from(
            bodyA.collisionShape.get(),
            bodyA.state.position,
            bodyA.state.orientation
        );
        const TransformedShape tB = TransformedShape::from(
            bodyB.collisionShape.get(),
            bodyB.state.position,
            bodyB.state.orientation
        );

        // ── Stage 1: GJK — fast boolean intersection ──────────────────────
        ++m_gjkTests;
        const GJKResult gjk = GJK::test(tA, tB);
        if (!gjk.collision) continue;
        ++m_gjkHits;

        // ── Stage 2: EPA — extract penetration data ────────────────────────
        Manifold manifold = EPA::solve(gjk.simplex, tA, tB);

        if (!manifold.isValid) {
            ++m_epaFails;
            continue;
        }

        manifold.bodyA = pair.a;
        manifold.bodyB = pair.b;
        m_manifolds.push_back(manifold);
    }
}