#pragma once
#include "RigidBody.hpp"
#include "collision/IConvexShape.hpp"
#include <glm/glm.hpp>
#include <memory>

namespace Inertia {
    inline glm::mat3 box(float mass, const glm::vec3& halfExtents) {
        const float hx = halfExtents.x * 2.0f;
        const float hy = halfExtents.y * 2.0f;
        const float hz = halfExtents.z * 2.0f;
        const float k = mass / 12.0f;
        return glm::inverse(glm::mat3(
            k * (hy*hy + hz*hz), 0.0f, 0.0f,
            0.0f, k * (hx*hx + hz*hz), 0.0f,
            0.0f, 0.0f, k * (hx*hx + hy*hy)
        ));
    }

    inline glm::mat3 sphere(float mass, float radius) {
        const float I = (2.0f / 5.0f) * mass * radius * radius;
        return glm::inverse(glm::mat3(I));
    }
}

namespace RigidBodyFactory {
    struct BodyBlueprint {
        RigidBodyState state;
        RigidBodyProperties properties;
        std::shared_ptr<IConvexShape> shape;
    };

    inline BodyBlueprint makeBox(const glm::vec3& position, const glm::vec3& halfExtents, float mass, float restitution = 0.3f, float friction = 0.5f) {
        RigidBodyState state;
        state.position = state.prevPosition = position;
        RigidBodyProperties props;
        props.inverseMass = 1.0f / mass;
        props.inverseInertiaTensorBody = Inertia::box(mass, halfExtents);
        props.boundingRadius = glm::length(halfExtents);
        props.restitution = restitution;
        props.friction = friction;
        return { state, props, std::make_shared<BoxShape>(halfExtents) };
    }

    inline BodyBlueprint makeStaticBox(const glm::vec3& position, const glm::vec3& halfExtents) {
        RigidBodyState state;
        state.position = state.prevPosition = position;
        RigidBodyProperties props;
        props.inverseMass = 0.0f; 
        props.inverseInertiaTensorBody = glm::mat3(0.0f);
        props.boundingRadius = glm::length(halfExtents);
        props.restitution = 0.2f;
        props.friction = 0.6f;
        return { state, props, std::make_shared<BoxShape>(halfExtents) };
    }
}