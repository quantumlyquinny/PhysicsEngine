#pragma once
#include "../spatial/Octree.hpp"
#include "BroadPhaseTypes.hpp"
#include "RigidBody.hpp"
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// BroadPhase — owns the Octree and drives the per-step pipeline:
//
//   1. Compute AABBs from body state (position + bounding radius + velocity)
//   2. Expand world bounds to contain all bodies (never shrinks in a session)
//   3. Rebuild the Octree
//   4. Collect overlapping candidate pairs
//
// Output: a pre-allocated vector of BodyPairs consumed by NarrowPhase.
// ─────────────────────────────────────────────────────────────────────────────

class BroadPhase {
public:
    explicit BroadPhase(std::size_t maxBodies);

    // Called once per physics step, before NarrowPhase.
    void update(const std::vector<RigidBody>& bodies, float dt);

    const std::vector<BodyPair>& getCandidatePairs() const { return m_pairs; }
    const AABB&                  getWorldBounds()    const { return m_worldBounds; }
    const Octree&                getOctree()         const { return m_octree; }

    // Stats for the debug overlay
    int getPairCount()  const { return static_cast<int>(m_pairs.size()); }
    int getNodeCount()  const { return m_octree.getNodeCount(); }
    int getTreeDepth()  const { return m_octree.getDepth(); }

private:
    Octree               m_octree;

    // Pre-allocated — never trigger reallocation after construction
    std::vector<BodyPair>  m_pairs;    // output: candidate pairs
    std::vector<BodyEntry> m_entries;  // scratch: one entry per live body

    // World AABB managed by BroadPhase, not the Octree.
    // Grows to contain all bodies; never shrinks within a session.
    // This prevents the root node bounds from thrashing when objects
    // cluster together and then spread apart.
    AABB m_worldBounds;

    static constexpr float WORLD_BOUNDS_MARGIN  = 5.0f;  // metres
    static constexpr float INITIAL_WORLD_RADIUS = 100.0f;

    // ── Per-step helpers ──────────────────────────────────────────────────
    AABB computeBodyAABB(const RigidBody& body, float dt) const;
    void expandWorldBounds(const std::vector<BodyEntry>& entries);
};