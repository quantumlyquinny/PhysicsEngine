#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>

using BodyID = std::uint32_t;
static constexpr BodyID INVALID_BODY_ID = 0xFFFFFFFF;

struct RigidBodyState {
    // ── Linear (Verlet: two positions, no explicit velocity) ──────────────
    glm::vec3 position        = glm::vec3(0.0f);
    glm::vec3 prevPosition    = glm::vec3(0.0f);

    // ── Angular (symplectic Euler: explicit angular velocity) ─────────────
    // angularVelocity lives in WORLD space. We transform the body-space
    // inertia tensor to world space each step — never store world-space
    // inertia permanently, it changes every frame as the body rotates.
    glm::quat orientation     = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::quat prevOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 angularVelocity = glm::vec3(0.0f); // world space, rad/s

    // ── Interpolated (renderer only — physics NEVER reads these) ──────────
    glm::vec3 renderPosition    = glm::vec3(0.0f);
    glm::quat renderOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
};

struct RigidBodyProperties {
    float     inverseMass              = 1.0f;   // 0.0f → infinite mass (static)
    glm::mat3 inverseInertiaTensorBody = glm::mat3(1.0f); // body-space, constant
    float     restitution              = 0.5f;
    float     linearDamping            = 0.005f; // small — corrects Verlet drift
    float     angularDamping           = 0.005f;
    float     friction                 = 0.4f;
};

struct RigidBodyForces {
    // Zeroed at the START of each physics step by the Integrator.
    // Every system (gravity, springs, user forces) ACCUMULATES into these.
    glm::vec3 accumForce  = glm::vec3(0.0f);
    glm::vec3 accumTorque = glm::vec3(0.0f);
};

class RigidBody {
public:
    BodyID              id         = INVALID_BODY_ID;
    RigidBodyState      state;
    RigidBodyProperties properties;
    RigidBodyForces     forces;
    bool                isStatic   = false;
    bool                isSleeping = false;
    float               sleepTimer = 0.0f;

    // ── Force application helpers ─────────────────────────────────────────
    void applyForce(const glm::vec3& force);

    // Apply force at a world-space offset from centre of mass.
    // Automatically splits into linear force + torque.
    void applyForceAtPoint(const glm::vec3& force, const glm::vec3& worldPoint);
    void applyTorque(const glm::vec3& torque);
    void clearForces();

    // ── Derived quantities ────────────────────────────────────────────────
    // Derive Verlet velocity on demand — NOT stored to avoid accumulation.
    // dt is the physics timestep used to produce the current position delta.
    glm::vec3 getLinearVelocity(float dt) const;

    // World-space inverse inertia tensor (recomputed from orientation).
    // Cheap — one mat3 multiply. Called by integrator and impulse solver.
    glm::mat3 getWorldInverseInertiaTensor() const;

    glm::mat4 getTransformMatrix() const;
};