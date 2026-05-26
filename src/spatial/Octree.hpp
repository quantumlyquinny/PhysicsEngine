#pragma once
#include "../physics/BroadPhaseTypes.hpp"
#include <array>
#include <vector>
#include <cassert>

// ─────────────────────────────────────────────────────────────────────────────
// Loose Octree — spatial partitioning for broad-phase collision culling.
//
// Key invariants:
//   • Each body is inserted into EXACTLY ONE node — the deepest node whose
//     bounds fully contain the body's AABB.
//   • Bodies whose AABB straddles an octant boundary stay in the PARENT node.
//     No duplication, no redistribution on subdivide.
//   • Subdivide is triggered when a LEAF node reaches SPLIT_THRESHOLD bodies
//     AND depth < MAX_DEPTH. Internal nodes never trigger a subdivide.
//   • The tree is rebuilt from scratch every physics step (zero allocations
//     because all storage is pre-allocated in the constructor).
// ─────────────────────────────────────────────────────────────────────────────

class Octree {
public:
    // ── Tunable constants ─────────────────────────────────────────────────
    // MAX_NODES: 4096 bodies, balanced tree ≈ 585 nodes. 4096 is generous.
    // MAX_BODIES_PER_NODE: applies to BOTH leaves (split trigger) and internal
    //   nodes (hard cap for straddling bodies). 32 handles all realistic cases.
    // SPLIT_THRESHOLD: a leaf subdivides when bodyCount hits this value.
    // MAX_DEPTH: 7 → minimum leaf cell = worldSize / 128. At 200m world that's
    //   ~1.5m per cell, appropriate for human-scale objects.
    // MAX_ANCESTOR_STACK: depth * MAX_BODIES_PER_NODE = 7*32 = 224. 512 is safe.
    static constexpr int MAX_NODES          = 4096;
    static constexpr int MAX_BODIES_PER_NODE = 32;
    static constexpr int SPLIT_THRESHOLD    = 8;
    static constexpr int MAX_DEPTH          = 7;
    static constexpr int MAX_ANCESTOR_STACK = 512;
    static constexpr int AABB_CACHE_SIZE    = 4096; // must match MAX_BODIES

    Octree();

    // Full rebuild from the provided body list.
    // worldBounds must enclose all entries — managed by BroadPhase.
    void build(const std::vector<BodyEntry>& entries, const AABB& worldBounds);

    // Traverse the tree, collect all candidate (AABB-overlapping) pairs.
    // outPairs is cleared then filled. Pre-reserve before calling.
    void collectPairs(std::vector<BodyPair>& outPairs) const;

    // Point/volume query — used for picking, trigger volumes, AI queries.
    // NOT called each frame; no performance constraint on this path.
    void query(const AABB& volume, std::vector<BodyID>& outIDs) const;

    int  getNodeCount()  const { return m_nodeCount; }
    int  getDepth()      const;
    AABB getWorldBounds() const { return m_nodes[0].bounds; }

private:
    // ── Node layout ───────────────────────────────────────────────────────
    // All storage is inline — no pointers, no heap. Cache-friendly traversal.
    struct Node {
        AABB   bounds;
        int    children[8];                      // -1 = child absent
        BodyID bodies[MAX_BODIES_PER_NODE];      // inline body storage
        int    bodyCount;
        int    depth;
        bool   isLeaf;

        void reset(const AABB& b, int d) noexcept {
            bounds    = b;
            depth     = d;
            isLeaf    = true;
            bodyCount = 0;
            for (auto& c : children) c = -1;
        }
    };

    // ── Pre-allocated storage (zero per-frame allocation after init) ──────
    std::array<Node, MAX_NODES> m_nodes;
    int m_nodeCount = 0;

    // Per-body info cached at build() time — eliminates lookups during traversal
    struct CachedInfo {
        AABB aabb;
        bool isStatic;
        bool isSleeping;
    };
    std::array<CachedInfo, AABB_CACHE_SIZE> m_cache;

    // ── Internal helpers ──────────────────────────────────────────────────
    int  allocNode();
    void insert(int nodeIdx, BodyID id, const AABB& aabb);
    void subdivide(int nodeIdx);
    bool addBodyToNode(int nodeIdx, BodyID id);    // returns false if full
    AABB childBounds(const AABB& parent, int octant) const noexcept;

    void collectPairsRecursive(int             nodeIdx,
                               BodyID*         ancestorStack,
                               int             ancestorCount,
                               std::vector<BodyPair>& outPairs) const;

    void queryRecursive(int nodeIdx, const AABB& volume,
                        std::vector<BodyID>& outIDs) const;

    // Checks the isStatic/isSleeping skip conditions, THEN the AABB overlap.
    // Only adds the pair if both tests pass.
    void tryAddPair(BodyID a, BodyID b, std::vector<BodyPair>& pairs) const;
};