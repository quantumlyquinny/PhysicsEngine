#include "PhysicsWorld.hpp"
#include "Integrator.hpp"
#include "../core/Timer.hpp"
#include "BroadPhase.hpp"
#include "collision/NarrowPhase.hpp"
#include "collision/Manifold.hpp"
#include "collision/ContactSolver.hpp"

constexpr std::size_t PhysicsWorld::MAX_BODIES;

PhysicsWorld::PhysicsWorld() {
    m_bodies.reserve(MAX_BODIES);
    m_freeList.reserve(MAX_BODIES);

    m_integrator    = std::make_unique<Integrator>();
    m_broadPhase    = std::make_unique<BroadPhase>(MAX_BODIES);
    m_narrowPhase   = std::make_unique<NarrowPhase>(MAX_BODIES * 8);
    m_contactSolver = std::make_unique<ContactSolver>(MAX_BODIES * 4);
}

PhysicsWorld::~PhysicsWorld() = default;

BodyID PhysicsWorld::createBody(const RigidBodyState& state,
                                const RigidBodyProperties& props,
                                bool isStatic)
{
    BodyID id;
    if (!m_freeList.empty()) {
        id = m_freeList.back();
        m_freeList.pop_back();
        m_bodies[id] = RigidBody{}; 
    } else {
        id = static_cast<BodyID>(m_bodies.size());
        m_bodies.emplace_back();
    }

    RigidBody& body = m_bodies[id];
    body.id         = id;
    body.state      = state;
    body.properties = props;
    body.isStatic   = isStatic;

    body.state.prevPosition    = state.position;
    body.state.prevOrientation = state.orientation;

    return id;
}

void PhysicsWorld::destroyBody(BodyID id) {
    if (id >= m_bodies.size()) return;
    m_bodies[id].id       = INVALID_BODY_ID; 
    m_bodies[id].isStatic = true;            
    m_freeList.push_back(id);
}

RigidBody* PhysicsWorld::getBody(BodyID id) {
    if (id >= m_bodies.size() || m_bodies[id].id == INVALID_BODY_ID)
        return nullptr;
    return &m_bodies[id];
}

void PhysicsWorld::step(float dt) {
    const float dtSq = dt * dt;

    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic) continue;
        Integrator::clearForces(body);
    }

    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic || body.isSleeping) continue;
        if (body.properties.inverseMass > 0.0f) {
            const float mass = 1.0f / body.properties.inverseMass;
            const glm::vec3 gravForce = m_gravity * mass;
            body.applyForce(gravForce);
        }
    }

    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic || body.isSleeping) continue;
        Integrator::integrate(body, dt, dtSq);
    }

    m_broadPhase->update(m_bodies, dt);
    const auto& candidatePairs = m_broadPhase->getCandidatePairs();

    m_narrowPhase->detectCollisions(m_bodies, candidatePairs);
    const auto& manifolds = m_narrowPhase->getManifolds();

    solveConstraints(manifolds, dt);
    updateSleepState(dt);
}

void PhysicsWorld::solveConstraints(const std::vector<Manifold>& manifolds, float dt) {
    m_contactSolver->solve(m_bodies, manifolds, dt);
}

void PhysicsWorld::updateSleepState(float dt) {
    static constexpr float SLEEP_ENERGY_THRESHOLD = 0.005f;
    static constexpr float SLEEP_TIME_THRESHOLD   = 0.5f;   

    for (auto& body : m_bodies) {
        if (body.id == INVALID_BODY_ID || body.isStatic) continue;

        const glm::vec3 vel = body.getLinearVelocity(dt);
        const float ke = glm::dot(vel, vel) + glm::dot(body.state.angularVelocity, body.state.angularVelocity);

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

void PhysicsWorld::setCollisionShape(BodyID id, std::shared_ptr<IConvexShape> shape) {
    if (RigidBody* body = getBody(id)) {
        body->collisionShape = std::move(shape);
        body->properties.boundingRadius = body->collisionShape->getBoundingRadius();
    }
}

void PhysicsWorld::setGravity(const glm::vec3& gravity) {
    m_gravity = gravity;
}