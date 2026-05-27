#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// A point on the Minkowski difference of two convex shapes.
struct SupportPoint {
    glm::vec3 pointA;     // World-space support point on shape A
    glm::vec3 pointB;     // World-space support point on shape B
    glm::vec3 minkowski;  // pointA - pointB  (point on the Minkowski difference)

    SupportPoint() : pointA(0.0f), pointB(0.0f), minkowski(0.0f) {}
    SupportPoint(const glm::vec3& a, const glm::vec3& b)
        : pointA(a), pointB(b), minkowski(a - b) {}
};

class IConvexShape; // forward-declared

// A convex shape paired with a world transform.
struct TransformedShape {
    const IConvexShape* shape    = nullptr;
    glm::vec3           position = glm::vec3(0.0f);
    glm::mat3           rotation = glm::mat3(1.0f);

    glm::vec3 support(const glm::vec3& worldDir) const;

    static TransformedShape from(const IConvexShape* shape,
                                 const glm::vec3&    position,
                                 const glm::quat&    orientation);
};