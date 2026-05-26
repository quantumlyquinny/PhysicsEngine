#include "PhysicsWorld.hpp"
#include "Integrator.hpp"
#include "../core/Timer.hpp"
#include "BroadPhase.hpp"

PhysicsWorld::PhysicsWorld() {
    // ── Pre-allocate all body storage upfront ─────────────────────────────
    // reserve() sets capacity without constructing objects.
    // The simulation loop will never trigger a reallocation.
    m_bodies.reserve(MAX_BODIES);
    m_freeList.reserve(MAX_BODIES);

    m_integrator  = std::make_unique<Integrator>();
    m_broadPhase  = std::make_unique<BroadPhase>(MAX_BODIES);// Step 3
// m_narrowPhase = std::make_unique<NarrowPhase>();  // Step 4
}

PhysicsWorld::~PhysicsWorld() = default;

// ─────────────────────────────────────────────────────────────────────────────
BodyID PhysicsWorld::createBody(const RigidBodyState&      state,
                                const RigidBodyProperties& props,
                                bool                       isStatic)
{
    BodyID id;

    if (!m_freeList.empty()) {
        // Recycle a slot — no allocation
        id = m_freeList.back();
        m_freeList.pop_back();
        m_bodies[id]            = RigidBody{};  // Reset to default state
    } else {
        // First use of this slot — vector grows, but only up to MAX_BODIES
        id = static_cast<BodyID>(m_bodies.size());
        m_bodies.emplace_back();
    }

    RigidBody& body       = m_bodies[id];
    body.id               = id;
    body.state            = state;
    body.properties       = props;
    body.isStatic         = isStatic;

    // Sync prevPosition to position so the first Verlet step
    // produces zero initial velocity (body starts at rest).
    body.state.prevPosition    = state.position;
    body.state.prevOrientation = state.orientation;

    return id;
}

// ─────────────────────────────────────────────────────────────────────────────
void PhysicsWorld::destroyBody(BodyID id) {
    if (id >= m_bodies.size()) return;
    m_bodies[id].id        = INVALID_BODY_ID; // Mark as dead
    m_bodies[id].isStatic  = true;            // Exclude from integration
    m_freeList.push_back(id);
    // No deallocation — the slot is reused by the next createBody() call.
}

RigidBody* PhysicsWorld::getBody(BodyID id) {
    if (id >= m_bodies.size() || m_bodies[id].id == INVALID_BODY_ID)
        return nullptr;
    return &m_bodies[id];
}

// ─────────────────────────────────────────────────────────────────────────────
//  THE STEP PIPELINE
//  Called by Engine::run() on a fixed 120Hz tick.
//  Execution order is not arbitrary — each stage depends on the previous.
// ─────────────────────────────────────────────────────────────────────────────
void PhysicsWorld::step(float dt) {
    const float dtSq = dt * dt;

    // ── Stage 1: Clear per-step force accumulators ────────────────────────
    // MUST happen before any force application.
    // Forces are ephemeral: gravity re-applies each step, springs
    // re-evaluate each step, user forces must re-apply each step.
    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic) continue;
        Integrator::clearForces(body);
    }

    // ── Stage 2: Apply persistent forces ─────────────────────────────────
    // Gravity is F = m · g = g / inverseMass
    // Guard against infinite-mass bodies (inverseMass == 0).
    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic || body.isSleeping)
            continue;
        if (body.properties.inverseMass > 0.0f) {
            const float mass          = 1.0f / body.properties.inverseMass;
            const glm::vec3 gravForce = m_gravity * mass;
            body.applyForce(gravForce);
        }
    }

    // ── Stage 3: Integrate — advance all bodies by dt ────────────────────
    // Reads:  accumForce, accumTorque, position, prevPosition,
    //         orientation, prevOrientation, angularVelocity
    // Writes: position, prevPosition (swapped), orientation,
    //         prevOrientation (swapped), angularVelocity
    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic || body.isSleeping)
            continue;
        Integrator::integrate(body, dt, dtSq);
    }

    // ── Stage 4: Broad Phase — cull to candidate collision pairs ─────────
    // Queries the Octree; fills m_candidatePairs with (BodyID, BodyID).
    // O(n log n) vs the O(n²) brute-force alternative.
    // STUB — implemented in Step 3.
    m_broadPhase->update(m_bodies, dt);
    const auto& candidatePairs = m_broadPhase->getCandidatePairs();

    // ── Stage 5: Narrow Phase — GJK + EPA per candidate pair ─────────────
    // For each pair: runs GJK (boolean), then EPA (contact manifold).
    // Fills m_manifolds with penetration depth + contact normal.
    // STUB — implemented in Step 4.
    //m_narrowPhase->detectCollisions(m_bodies, candidatePairs);
    //const auto& manifolds = m_narrowPhase->getManifolds();

    // ── Stage 6: Constraint & Contact Solver ─────────────────────────────
    // Applies velocity-level impulses to resolve penetrations.
    // Modifies prevPosition (not position) to correct Verlet's displacement.
    // STUB — implemented alongside Step 4.
    //solveConstraints(manifolds, dt);

    // ── Stage 7: Sleep Classification ────────────────────────────────────
    // Computes kinetic energy estimate; marks bodies as sleeping when
    // below threshold for N consecutive steps. Sleeping bodies skip
    // Stages 1–3 entirely, saving significant CPU time in settled scenes.
    updateSleepState(dt);
}

// ─────────────────────────────────────────────────────────────────────────────
void PhysicsWorld::solveConstraints(
    const std::vector<Manifold>& manifolds, float dt)
{
    // STUB — see Step 4 (Contact Solver / impulse resolution)
    (void)manifolds; (void)dt;
}

void PhysicsWorld::updateSleepState(float dt) {
    // Threshold: body is sleeping candidate if KE < epsilon for > 0.5s
    static constexpr float SLEEP_ENERGY_THRESHOLD = 0.005f;
    static constexpr float SLEEP_TIME_THRESHOLD   = 0.5f;   // seconds

    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic) continue;

        // KE proxy: |v|² + |ω|² (skips mass — relative comparison only)
        const glm::vec3 vel  = body.getLinearVelocity(dt);
        const float     ke   = glm::dot(vel, vel)
                             + glm::dot(body.state.angularVelocity,
                                        body.state.angularVelocity);

        if (ke < SLEEP_ENERGY_THRESHOLD) {
            body.sleepTimer += dt;
            if (body.sleepTimer > SLEEP_TIME_THRESHOLD) {
                body.isSleeping = true;
            }
        } else {
            body.sleepTimer = 0.0f;
            body.isSleeping = false;
        }
    }
}