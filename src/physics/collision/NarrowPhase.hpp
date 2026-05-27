// NarrowPhase.hpp
#pragma once
#include "GJK.hpp"
#include "EPA.hpp"
#include "Manifold.hpp"
#include "../BroadPhaseTypes.hpp"
#include "../RigidBody.hpp"
#include <vector>
// ─────────────────────────────────────────────────────────────────────────────
// NarrowPhase — processes candidate pairs from BroadPhase.
//
// Per pair:
//   1. GJK: boolean intersection test (fast O(1) average case)
//   2. EPA: contact normal + penetration depth (only if GJK reports collision)
//   3. Assembles Manifold → consumed by ContactSolver
//
// Zero allocations per frame — m_manifolds is pre-reserved at construction.
// ─────────────────────────────────────────────────────────────────────────────
class NarrowPhase {
public:
    explicit NarrowPhase(std::size_t maxExpectedPairs);

    void detectCollisions(const std::vector<RigidBody>& bodies,
                          const std::vector<BodyPair>&  pairs);

    const std::vector<Manifold>& getManifolds() const { return m_manifolds; }

    // Per-frame stats for the debug overlay
    int getGJKTestCount()    const { return m_gjkTests;     }
    int getGJKHitCount()     const { return m_gjkHits;      }
    int getEPAFailCount()    const { return m_epaFails;     }
    int getManifoldCount()   const { return static_cast<int>(m_manifolds.size()); }

private:
    std::vector<Manifold> m_manifolds;

    // Per-frame counters (reset in detectCollisions)
    int m_gjkTests  = 0;
    int m_gjkHits   = 0;
    int m_epaFails  = 0;
};