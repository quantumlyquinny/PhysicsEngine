#include "RigidBody.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

// ─────────────────────────────────────────────────────────────────────────────
void RigidBody::applyForce(const glm::vec3& force) {
    if (isStatic) return;
    forces.accumForce += force;
}

void RigidBody::applyForceAtPoint(const glm::vec3& force,
                                  const glm::vec3& worldPoint) {
    if (isStatic) return;
    forces.accumForce  += force;
    // r × F — offset from centre of mass to the application point
    const glm::vec3 r   = worldPoint - state.position;
    forces.accumTorque += glm::cross(r, force);
}

void RigidBody::applyTorque(const glm::vec3& torque) {
    if (isStatic) return;
    forces.accumTorque += torque;
}

void RigidBody::clearForces() {
    forces.accumForce  = glm::vec3(0.0f);
    forces.accumTorque = glm::vec3(0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
glm::vec3 RigidBody::getLinearVelocity(float dt) const {
    // Central-difference approximation of velocity from Verlet positions.
    // v ≈ (x_cur − x_prev) / dt
    // This is valid for inter-step queries (e.g. sleeping detection,
    // velocity-based friction). The impulse solver uses this directly.
    if (dt < 1e-7f) return glm::vec3(0.0f);
    return (state.position - state.prevPosition) / dt;
}

// ─────────────────────────────────────────────────────────────────────────────
glm::mat3 RigidBody::getWorldInverseInertiaTensor() const {
    // Convert orientation quaternion to a 3×3 rotation matrix.
    const glm::mat3 R = glm::toMat3(state.orientation);
    // I_world⁻¹ = R · I_body⁻¹ · Rᵀ
    // This rotates the body-space (diagonal) tensor into world space.
    // We recompute this every call — it's 18 multiplications and cheaper
    // than caching it and managing invalidation.
    return R * properties.inverseInertiaTensorBody * glm::transpose(R);
}

// ─────────────────────────────────────────────────────────────────────────────
glm::mat4 RigidBody::getTransformMatrix() const {
    // Build from interpolated render state so the renderer always gets
    // the smoothed transform, never the raw physics position.
    const glm::mat4 T = glm::translate(glm::mat4(1.0f), state.renderPosition);
    const glm::mat4 R = glm::toMat4(state.renderOrientation);
    return T * R;
    // Scale is intentionally omitted — collision geometry owns its own scale.
}