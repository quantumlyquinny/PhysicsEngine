#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cfloat>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// IConvexShape — the single interface GJK and EPA require from any geometry.
// ─────────────────────────────────────────────────────────────────────────────
class IConvexShape {
public:
    virtual ~IConvexShape() = default;

    // Renamed to getSupport to match GJK/EPA requirements
    virtual glm::vec3 getSupport(const glm::vec3& direction) const = 0;

    // Conservative bounding sphere radius in local space.
    virtual float getBoundingRadius() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Sphere — analytical support, O(1)
// ─────────────────────────────────────────────────────────────────────────────
class SphereShape final : public IConvexShape {
public:
    explicit SphereShape(float r) : m_radius(r) {}

    glm::vec3 getSupport(const glm::vec3& dir) const override {
        const float lenSq = glm::dot(dir, dir);
        if (lenSq < 1e-10f) return glm::vec3(m_radius, 0.0f, 0.0f);
        return dir * (m_radius / std::sqrt(lenSq));
    }

    float getBoundingRadius() const override { return m_radius; }

private:
    float m_radius;
};

// ─────────────────────────────────────────────────────────────────────────────
// Box (OBB half-extents) — analytical support, O(1)
// ─────────────────────────────────────────────────────────────────────────────
class BoxShape final : public IConvexShape {
public:
    explicit BoxShape(const glm::vec3& halfExtents) : m_half(halfExtents) {}

    glm::vec3 getSupport(const glm::vec3& dir) const override {
        return glm::vec3(
            dir.x >= 0.0f ?  m_half.x : -m_half.x,
            dir.y >= 0.0f ?  m_half.y : -m_half.y,
            dir.z >= 0.0f ?  m_half.z : -m_half.z
        );
    }

    float getBoundingRadius() const override { return glm::length(m_half); }

private:
    glm::vec3 m_half;
};

// ─────────────────────────────────────────────────────────────────────────────
// ConvexHull — brute-force O(n) support over a vertex cloud
// ─────────────────────────────────────────────────────────────────────────────
class ConvexHullShape final : public IConvexShape {
public:
    void build(const std::vector<glm::vec3>& verts) {
        m_vertices = verts;
        m_boundingRadius = 0.0f;
        for (const auto& v : m_vertices) {
            const float r = glm::length(v);
            if (r > m_boundingRadius) m_boundingRadius = r;
        }
    }

    glm::vec3 getSupport(const glm::vec3& dir) const override {
        float     bestDot = -FLT_MAX;
        glm::vec3 best    = m_vertices.empty() ? glm::vec3(0.0f) : m_vertices[0];

        for (const auto& v : m_vertices) {
            const float d = glm::dot(v, dir);
            if (d > bestDot) { bestDot = d; best = v; }
        }
        return best;
    }

    float getBoundingRadius() const override { return m_boundingRadius; }

private:
    std::vector<glm::vec3> m_vertices;
    float                  m_boundingRadius = 0.0f;
};