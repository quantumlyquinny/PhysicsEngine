#pragma once
#include <glm/glm.hpp>

struct AABB {
    glm::vec3 min = glm::vec3( 1e30f);
    glm::vec3 max = glm::vec3(-1e30f);

    static AABB fromCenterHalfExtents(const glm::vec3& c, const glm::vec3& h) {
        return { c - h, c + h };
    }

    bool overlaps(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }

    bool contains(const glm::vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    AABB expanded(float margin) const {
        glm::vec3 m(margin);
        return { min - m, max + m };
    }

    glm::vec3 center()      const { return (min + max) * 0.5f; }
    glm::vec3 halfExtents() const { return (max - min) * 0.5f; }
};