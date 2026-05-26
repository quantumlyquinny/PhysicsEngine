#include "Integrator.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

// ─────────────────────────────────────────────────────────────────────────────
void Integrator::integrate(RigidBody& body, float dt, float dtSq) {
    // Static and sleeping bodies contribute nothing — skip entirely.
    if (body.isStatic || body.isSleeping) return;

    // ── Linear Integration (Position Verlet) ─────────────────────────────
    {
        // Linear acceleration this step: a = F / m
        // inverseMass = 0 means infinite mass; force produces no acceleration.
        const glm::vec3 acceleration = body.forces.accumForce
                                     * body.properties.inverseMass;

        // Save the current position into prevPosition BEFORE overwriting it.
        // This is the position from the last step — Verlet needs it.
        const glm::vec3 currentPos = body.state.position;

        // Verlet core:  x_new = 2·x_cur − x_prev + a·dt²
        // Damping term: multiply the displacement by (1 − damping) to bleed
        // a small fraction of kinetic energy each step.
        const float     dampFactor  = 1.0f - body.properties.linearDamping;
        const glm::vec3 displacement = (currentPos - body.state.prevPosition)
                                       * dampFactor;

        body.state.position     = currentPos + displacement + acceleration * dtSq;
        body.state.prevPosition = currentPos;
    }

    // ── Angular Integration (Symplectic Euler) ────────────────────────────
    {
        // Compute the world-space inverse inertia tensor for this orientation.
        // I_world⁻¹ = R · I_body⁻¹ · Rᵀ
        // where R is the rotation matrix derived from the body's quaternion.
        const glm::mat3 invI_world = body.getWorldInverseInertiaTensor();

        // Angular acceleration: α = I⁻¹ · τ
        const glm::vec3 angularAccel = invI_world * body.forces.accumTorque;

        // Symplectic Euler step 1: update angular velocity FIRST
        // Apply angular damping analogous to linear damping.
        body.state.angularVelocity += angularAccel * dt;
        body.state.angularVelocity *= (1.0f - body.properties.angularDamping);

        // Symplectic Euler step 2: update orientation from new angular velocity
        // Quaternion derivative: dq/dt = ½ · ω_quat · q
        // where ω_quat = quaternion(0, ω.x, ω.y, ω.z)
        const glm::quat prevOrient = body.state.orientation;

        const glm::quat omegaQuat(
            0.0f,
            body.state.angularVelocity.x,
            body.state.angularVelocity.y,
            body.state.angularVelocity.z
        );

        // Integrate: q_new = q + 0.5 · ω_quat · q · dt
        body.state.orientation = body.state.orientation
                               + (omegaQuat * body.state.orientation)
                               * (0.5f * dt);

        // Quaternions drift from unit length under repeated addition —
        // normalise every step to prevent shearing artifacts.
        body.state.orientation     = glm::normalize(body.state.orientation);
        body.state.prevOrientation = prevOrient;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void Integrator::clearForces(RigidBody& body) {
    body.forces.accumForce  = glm::vec3(0.0f);
    body.forces.accumTorque = glm::vec3(0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
void Integrator::setPosition(RigidBody& body, const glm::vec3& pos) {
    // Setting both current AND previous prevents a single-frame velocity spike.
    body.state.position     = pos;
    body.state.prevPosition = pos;
}

void Integrator::setOrientation(RigidBody& body, const glm::quat& orient) {
    body.state.orientation     = glm::normalize(orient);
    body.state.prevOrientation = body.state.orientation;
}

void Integrator::setLinearVelocity(RigidBody& body,
                                   const glm::vec3& vel,
                                   float dt) {
    // Back-project prevPosition so that the Verlet displacement
    // on the NEXT step equals exactly (vel * dt).
    // prevPos = pos − vel · dt
    body.state.prevPosition = body.state.position - vel * dt;
}