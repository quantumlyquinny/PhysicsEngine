// In DebugDraw.hpp — add this method
//void drawOctree(const Octree& octree, const glm::mat4& viewProj);
//void drawBodyAABBs(const std::vector<RigidBody>& bodies,
//                   const glm::mat4& viewProj);
//void drawCandidatePairs(const std::vector<BodyPair>& pairs,
//                        const std::vector<RigidBody>& bodies,
//                        const glm::mat4& viewProj);

// ── In DebugDraw.cpp — sketch implementation ─────────────────────────────────
//void DebugDraw::drawOctree(const Octree& octree, const glm::mat4& viewProj) {
    // Walk every node in the pool and draw its AABB wireframe.
    // Colour by depth: root = white, deeper = cooler blue.
//    for (int i = 0; i < octree.getNodeCount(); i++) {
        // Access via a friend declaration or a public node iterator
        // (add getNode(int) → const Node& to Octree if needed)
//    }
//}