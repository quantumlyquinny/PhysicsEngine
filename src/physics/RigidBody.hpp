#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>

// Strong ID type — never use raw pointers to reference bodies externally
using BodyID = std::uint32_t;
static constexpr BodyID INVALID_BODY_ID = 0xFFFFFFFF;

struct RigidBodyState {
    // --- Current state (Verlet needs TWO positions, not pos+vel) ---
    glm::vec3 position        = glm::vec3(0.0f);
    glm::vec3 prevPosition    = glm::vec3(0.0f); // Verlet previous position
    glm::quat orientation     = glm::quat(1,0,0,0);
    glm::quat prevOrientation = glm::quat(1,0,0,0);

    // --- Interpolated state (written by renderer, NEVER by physics) ---
    glm::vec3 renderPosition    = glm::vec3(0.0f);
    glm::quat renderOrientation = glm::quat(1,0,0,0);
};

struct RigidBodyProperties {
    float inverseMass    = 1.0f;   // 0 = static/infinite mass
    glm::mat3 inverseInertiaTensor = glm::mat3(1.0f);
    float restitution    = 0.5f;
    float friction       = 0.4f;
};

struct RigidBodyForces {
    glm::vec3 accumForce  = glm::vec3(0.0f);  // Zeroed each step
    glm::vec3 accumTorque = glm::vec3(0.0f);
};

// The body itself is a thin aggregation — all logic lives in systems
class RigidBody {
public:
    BodyID              id         = INVALID_BODY_ID;
    RigidBodyState      state;
    RigidBodyProperties properties;
    RigidBodyForces     forces;
    bool                isStatic   = false;
    bool                isSleeping = false;

    void applyForce(const glm::vec3& force);
    void applyTorque(const glm::vec3& torque);
    void clearForces(); // Called by Integrator at start of each step
    glm::mat4 getTransformMatrix() const;
};