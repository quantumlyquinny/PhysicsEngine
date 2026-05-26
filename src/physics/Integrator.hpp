#pragma once
#include "../physics/RigidBody.hpp"
#include <glm/glm.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Integrator — Stateless system. Operates on a single RigidBody per call.
//
// Linear:  Position Verlet
//   x(t+dt) = 2·x(t) − x(t−dt) + a(t)·dt²
//   a(t)    = F_accum / mass  (gravity already baked into F_accum by caller)
//
// Angular: Symplectic (semi-implicit) Euler
//   α       = I_world⁻¹ · τ_accum
//   ω(t+dt) = ω(t) + α·dt                   (velocity update first)
//   q(t+dt) = normalise(q(t) + ½·[0,ω]·q·dt) (orientation update second)
//
// Damping is applied as a velocity-proportional drag on both linear and
// angular motion to prevent energy accumulation from floating-point error.
// ─────────────────────────────────────────────────────────────────────────────

class Integrator {
public:
    // Integrate a single body forward by dt seconds.
    // Forces/torques must already be accumulated before this call.
    // prevPosition/prevOrientation are updated HERE, after integration,
    // so they always represent the state from the previous step.
    static void integrate(RigidBody& body, float dt, float dtSq);

    // Zeroes accumForce/accumTorque. Called at the TOP of each step,
    // before any force application. Forces are per-step, not persistent.
    static void clearForces(RigidBody& body);

    // Teleport a body to a new position without triggering false velocities.
    // Resets prevPosition to match — used for spawning and resetting.
    static void setPosition(RigidBody& body, const glm::vec3& pos);
    static void setOrientation(RigidBody& body, const glm::quat& orient);
    static void setLinearVelocity(RigidBody& body, const glm::vec3& vel, float dt);
};