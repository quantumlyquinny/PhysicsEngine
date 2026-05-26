#include "Octree.hpp"
#include <algorithm>  // std::sort, std::unique

Octree::Octree() {
    // std::array value-initialises its elements, so m_nodes is zeroed.
    // We still call reset() explicitly in allocNode for clarity.
}

// ─────────────────────────────────────────────────────────────────────────────
int Octree::allocNode() {
    assert(m_nodeCount < MAX_NODES
        && "Octree node pool exhausted — increase MAX_NODES");
    return m_nodeCount++;
}

// ─────────────────────────────────────────────────────────────────────────────
AABB Octree::childBounds(const AABB& parent, int octant) const noexcept {
    // Octant index encodes the XYZ half-space selection in bits 0, 1, 2.
    //   bit 0 set → use [centre.x, max.x]   else use [min.x, centre.x]
    //   bit 1 set → use [centre.y, max.y]   else use [min.y, centre.y]
    //   bit 2 set → use [centre.z, max.z]   else use [min.z, centre.z]
    const glm::vec3 c = parent.center();
    return {
        glm::vec3{
            (octant & 1) ? c.x : parent.min.x,
            (octant & 2) ? c.y : parent.min.y,
            (octant & 4) ? c.z : parent.min.z
        },
        glm::vec3{
            (octant & 1) ? parent.max.x : c.x,
            (octant & 2) ? parent.max.y : c.y,
            (octant & 4) ? parent.max.z : c.z
        }
    };
}

// ─────────────────────────────────────────────────────────────────────────────
bool Octree::addBodyToNode(int nodeIdx, BodyID id) {
    Node& node = m_nodes[nodeIdx];
    if (node.bodyCount >= MAX_BODIES_PER_NODE) {
        // This should not occur in well-configured scenes.
        // If it fires consistently, increase MAX_BODIES_PER_NODE
        // or reduce object sizes relative to world bounds.
        assert(false && "Octree node body overflow");
        return false;
    }
    node.bodies[node.bodyCount++] = id;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::subdivide(int nodeIdx) {
    // IMPORTANT: We must NOT hold a reference to m_nodes[nodeIdx] across
    // allocNode() calls — allocNode() increments m_nodeCount but since
    // m_nodes is a std::array (not a vector) it never reallocates. The
    // reference is stable; the comment is a reminder for future refactors
    // if the storage type changes.

    Node& node = m_nodes[nodeIdx];
    assert(node.isLeaf);
    node.isLeaf = false;

    for (int i = 0; i < 8; i++) {
        int childIdx = allocNode();
        m_nodes[childIdx].reset(childBounds(m_nodes[nodeIdx].bounds, i),
                                m_nodes[nodeIdx].depth + 1);
        m_nodes[nodeIdx].children[i] = childIdx;
    }
    // Existing bodies in this node are NOT redistributed.
    // They stay in the (now internal) node and are treated as
    // "straddling" bodies — tested against all descendants during
    // collectPairsRecursive via the ancestor stack.
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::insert(int nodeIdx, BodyID id, const AABB& aabb) {
    // ── Terminal cases ────────────────────────────────────────────────────
    if (m_nodes[nodeIdx].depth >= MAX_DEPTH) {
        addBodyToNode(nodeIdx, id);
        return;
    }

    if (m_nodes[nodeIdx].isLeaf) {
        if (m_nodes[nodeIdx].bodyCount < SPLIT_THRESHOLD) {
            // Leaf has room — store here
            addBodyToNode(nodeIdx, id);
            return;
        }
        // Leaf is full — subdivide, then fall through to the descent logic
        subdivide(nodeIdx);
    }

    // ── Internal node: descend into the one child that fully contains aabb ──
    // Re-fetch node ref after subdivide (safe with std::array, see comment above)
    for (int i = 0; i < 8; i++) {
        const int childIdx = m_nodes[nodeIdx].children[i];
        if (childIdx != -1 && m_nodes[childIdx].bounds.contains(aabb)) {
            insert(childIdx, id, aabb);
            return;
        }
    }

    // ── No single child contains this AABB — it straddles a boundary ─────
    // Store in this internal node. It will be matched against all descendant
    // bodies via the ancestor stack in collectPairsRecursive.
    addBodyToNode(nodeIdx, id);
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::build(const std::vector<BodyEntry>& entries, const AABB& worldBounds) {
    // ── Reset pools — O(1), no deallocation ──────────────────────────────
    m_nodeCount = 0;

    // ── Cache per-body AABB and flags indexed by BodyID ──────────────────
    for (const auto& e : entries) {
        assert(e.id < AABB_CACHE_SIZE
            && "BodyID exceeds AABB_CACHE_SIZE — increase to match MAX_BODIES");
        m_cache[e.id] = { e.aabb, e.isStatic, e.isSleeping };
    }

    // ── Create root and insert all bodies ────────────────────────────────
    const int root = allocNode();
    m_nodes[root].reset(worldBounds, 0);

    for (const auto& e : entries) {
        insert(root, e.id, e.aabb);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::tryAddPair(BodyID a, BodyID b,
                        std::vector<BodyPair>& pairs) const
{
    if (a == b) return;

    const CachedInfo& infoA = m_cache[a];
    const CachedInfo& infoB = m_cache[b];

    // Two static bodies never collide — skip immediately
    if (infoA.isStatic   && infoB.isStatic)   return;
    // Two sleeping bodies have no relative motion — skip
    if (infoA.isSleeping && infoB.isSleeping) return;

    // AABB overlap test — the core broad-phase filter
    if (!infoA.aabb.overlaps(infoB.aabb)) return;

    // Enforce canonical ordering (smaller ID first) to prevent duplicates
    const BodyID lo = (a < b) ? a : b;
    const BodyID hi = (a < b) ? b : a;
    pairs.push_back({ lo, hi });
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::collectPairsRecursive(int             nodeIdx,
                                   BodyID*         ancestorStack,
                                   int             ancestorCount,
                                   std::vector<BodyPair>& outPairs) const
{
    const Node& node = m_nodes[nodeIdx];

    // ── Cross-level: this node's bodies vs all ancestor bodies ───────────
    // This catches pairs where one body is in an ancestor (straddler) and
    // the other is in a descendant leaf.
    for (int i = 0; i < node.bodyCount; i++) {
        for (int j = 0; j < ancestorCount; j++) {
            tryAddPair(node.bodies[i], ancestorStack[j], outPairs);
        }
    }

    // ── Same-level: bodies within this node vs each other ────────────────
    for (int i = 0; i < node.bodyCount; i++) {
        for (int j = i + 1; j < node.bodyCount; j++) {
            tryAddPair(node.bodies[i], node.bodies[j], outPairs);
        }
    }

    // ── Recurse into children ─────────────────────────────────────────────
    if (!node.isLeaf) {
        // Push this node's bodies onto the ancestor stack before descending.
        // Since ancestorStack is a raw array and ancestorCount is passed by
        // value, modifications are local — the "pop" is implicit on return.
        assert(ancestorCount + node.bodyCount <= MAX_ANCESTOR_STACK
            && "Ancestor stack overflow — increase MAX_ANCESTOR_STACK");

        const int newAncestorCount = ancestorCount + node.bodyCount;
        for (int i = 0; i < node.bodyCount; i++) {
            ancestorStack[ancestorCount + i] = node.bodies[i];
        }

        for (int i = 0; i < 8; i++) {
            if (node.children[i] != -1) {
                collectPairsRecursive(node.children[i],
                                      ancestorStack, newAncestorCount,
                                      outPairs);
            }
        }
        // ancestorCount unchanged here — the pushed bodies are effectively
        // popped when we return to the caller's scope.
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::collectPairs(std::vector<BodyPair>& outPairs) const {
    outPairs.clear();

    if (m_nodeCount == 0) return;

    // Ancestor stack — allocated once on the call stack, reused across
    // all recursive calls. No heap activity.
    BodyID ancestorStack[MAX_ANCESTOR_STACK];
    collectPairsRecursive(0, ancestorStack, 0, outPairs);

    // Sort and deduplicate. Duplicates arise when a body straddles a boundary
    // at multiple ancestor levels, causing it to appear in several tryAddPair
    // calls with the same partner. std::sort is in-place; erase shrinks but
    // does not reallocate.
    std::sort(outPairs.begin(), outPairs.end());
    outPairs.erase(
        std::unique(outPairs.begin(), outPairs.end()),
        outPairs.end()
    );
}

// ─────────────────────────────────────────────────────────────────────────────
void Octree::queryRecursive(int nodeIdx, const AABB& volume,
                            std::vector<BodyID>& outIDs) const
{
    const Node& node = m_nodes[nodeIdx];
    if (!node.bounds.overlaps(volume)) return;

    for (int i = 0; i < node.bodyCount; i++) {
        if (m_cache[node.bodies[i]].aabb.overlaps(volume)) {
            outIDs.push_back(node.bodies[i]);
        }
    }

    if (!node.isLeaf) {
        for (int i = 0; i < 8; i++) {
            if (node.children[i] != -1) {
                queryRecursive(node.children[i], volume, outIDs);
            }
        }
    }
}

void Octree::query(const AABB& volume, std::vector<BodyID>& outIDs) const {
    outIDs.clear();
    if (m_nodeCount > 0) queryRecursive(0, volume, outIDs);
}

int Octree::getDepth() const {
    int maxDepth = 0;
    for (int i = 0; i < m_nodeCount; i++) {
        if (m_nodes[i].depth > maxDepth) maxDepth = m_nodes[i].depth;
    }
    return maxDepth;
}