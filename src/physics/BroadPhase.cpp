#include "BroadPhase.hpp"
#include <glm/glm.hpp>

// ─────────────────────────────────────────────────────────────────────────────
BroadPhase::BroadPhase(std::size_t maxBodies) {
    // Pre-allocate all working memory during construction.
    // No heap activity occurs inside update() after this point.
    m_pairs.reserve(maxBodies * 8);   // generous: 8 candidate pairs per body
    m_entries.reserve(maxBodies);

    // Start with a conservative world AABB. expandWorldBounds() will grow
    // this on the first step to exactly fit the simulation content.
    m_worldBounds = AABB::fromCenterHalfExtents(
        glm::vec3(0.0f),
        glm::vec3(INITIAL_WORLD_RADIUS)
    );
}

// ─────────────────────────────────────────────────────────────────────────────
AABB BroadPhase::computeBodyAABB(const RigidBody& body, float dt) const {
    // ── Conservative sphere AABB ──────────────────────────────────────────
    // Uses the body's bounding radius for a tight sphere-derived AABB.
    // Step 4 (GJK) will replace this with a support-function derived
    // tight AABB once we have actual convex mesh geometry.
    const float r = body.properties.boundingRadius;

    // ── Speculative contact expansion ────────────────────────────────────
    // Inflate the AABB in the direction of motion by the displacement
    // magnitude. This catches fast-moving collisions that would otherwise
    // tunnel through geometry entirely within a single physics step.
    //
    // displacement = position − prevPosition  (the Verlet step displacement)
    // We do NOT use velocity directly — displacement is already the truth.
    const glm::vec3 displacement = body.state.position - body.state.prevPosition;
    const float     speed        = glm::length(displacement);

    // Total half-extents: bounding radius + speculative expansion
    const float halfExtent = r + speed;

    // The AABB is centred on the current position (not the midpoint of the
    // displacement) because the narrow phase will resolve from current state.
    return AABB::fromCenterHalfExtents(
        body.state.position,
        glm::vec3(halfExtent)
    );
}

// ─────────────────────────────────────────────────────────────────────────────
void BroadPhase::expandWorldBounds(const std::vector<BodyEntry>& entries) {
    // Compute the tightest AABB around all body entries this step
    AABB frameBounds;
    for (const auto& e : entries) {
        frameBounds.min = glm::min(frameBounds.min, e.aabb.min);
        frameBounds.max = glm::max(frameBounds.max, e.aabb.max);
    }

    if (!frameBounds.isValid()) return;

    // Add a margin so the root node is never exactly flush with a body.
    // Bodies flush with the root boundary can straddle and fall to the root,
    // defeating the purpose of spatial partitioning.
    frameBounds.min -= glm::vec3(WORLD_BOUNDS_MARGIN);
    frameBounds.max += glm::vec3(WORLD_BOUNDS_MARGIN);

    // Grow-only: never shrink. This stabilises the octree structure across
    // frames and prevents repeated root resizes when objects oscillate.
    m_worldBounds.min = glm::min(m_worldBounds.min, frameBounds.min);
    m_worldBounds.max = glm::max(m_worldBounds.max, frameBounds.max);
}

// ─────────────────────────────────────────────────────────────────────────────
void BroadPhase::update(const std::vector<RigidBody>& bodies, float dt) {
    // ── Stage A: Build BodyEntry list ─────────────────────────────────────
    // clear() resets size to 0 but retains capacity — zero reallocation.
    m_entries.clear();

    for (const auto& body : bodies) {
        if (body.id == INVALID_BODY_ID) continue;

        m_entries.push_back({
            body.id,
            computeBodyAABB(body, dt),
            body.isStatic,
            body.isSleeping
        });
    }

    // ── Stage B: Grow world bounds if needed ──────────────────────────────
    expandWorldBounds(m_entries);

    // ── Stage C: Rebuild Octree ───────────────────────────────────────────
    // Full rebuild each step. O(n log n).
    // Incremental updates are faster in theory but introduce subtle correctness
    // issues when bodies are destroyed or teleported. Full rebuild is simpler
    // to reason about and fast enough for 4096 bodies at 120Hz.
    m_octree.build(m_entries, m_worldBounds);

    // ── Stage D: Collect candidate pairs ─────────────────────────────────
    // Traversal output goes into m_pairs (pre-allocated, cleared inside).
    m_octree.collectPairs(m_pairs);
}