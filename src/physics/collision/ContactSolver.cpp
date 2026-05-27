#include "ContactSolver.hpp"
#include <cmath>
#include <algorithm>

ContactSolver::ContactSolver(std::size_t maxContacts) {
    m_states.reserve(maxContacts);
}

float ContactSolver::effectiveMassContribution(const RigidBody& body, const glm::vec3& arm, const glm::vec3& dir) {
    if (body.isStatic || body.properties.inverseMass == 0.0f) return 0.0f;
    const glm::mat3 invI = body.getWorldInverseInertiaTensor();
    const glm::vec3 rCrossD = glm::cross(arm, dir);
    return body.properties.inverseMass + glm::dot(dir, glm::cross(invI * rCrossD, arm));
}

glm::vec3 ContactSolver::pointVelocity(const RigidBody& body, const glm::vec3& arm, float dt) {
    const glm::vec3 vLinear = (body.state.position - body.state.prevPosition) / dt;
    return vLinear + glm::cross(body.state.angularVelocity, arm);
}

glm::vec3 ContactSolver::relativeVelocity(const RigidBody& bodyA, const RigidBody& bodyB, const glm::vec3& rA, const glm::vec3& rB, float dt) {
    return pointVelocity(bodyA, rA, dt) - pointVelocity(bodyB, rB, dt);
}

void ContactSolver::applyImpulse(RigidBody& body, const glm::vec3& impulse, const glm::vec3& arm, float dt) {
    if (body.isStatic) return;
    
    // Linear: prevPosition carries the implicit Verlet velocity
    body.state.prevPosition -= (body.properties.inverseMass * dt) * impulse;
    
    // Angular: direct velocity update
    const glm::mat3 invI = body.getWorldInverseInertiaTensor();
    body.state.angularVelocity += invI * glm::cross(arm, impulse);
}

void ContactSolver::computeTangentBasis(const glm::vec3& n, glm::vec3& t1, glm::vec3& t2) {
    const glm::vec3 helper = (std::abs(n.x) < 0.5774f) ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    t1 = glm::normalize(glm::cross(n, helper));
    t2 = glm::cross(n, t1);
}

void ContactSolver::wake(RigidBody& body) {
    if (body.isSleeping) {
        body.isSleeping = false;
        body.sleepTimer = 0.0f;
    }
}

void ContactSolver::preStep(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt) {
    m_states.clear();
    for (const Manifold& m : manifolds) {
        if (!m.isValid) continue;
        if (m.bodyA >= bodies.size() || m.bodyB >= bodies.size()) continue;
        
        RigidBody& bodyA = bodies[m.bodyA];
        RigidBody& bodyB = bodies[m.bodyB];
        
        if (bodyA.isStatic && bodyB.isStatic) continue;
        
        wake(bodyA);
        wake(bodyB);
        
        ContactState cs;
        cs.bodyA = &bodyA;
        cs.bodyB = &bodyB;
        cs.normal = m.normal;
        
        cs.rA = m.contactPointA - bodyA.state.position;
        cs.rB = m.contactPointB - bodyB.state.position;
        
        computeTangentBasis(cs.normal, cs.tangent1, cs.tangent2);
        
        const auto buildEffMass = [&](const glm::vec3& dir) -> float {
            const float eA = effectiveMassContribution(bodyA, cs.rA, dir);
            const float eB = effectiveMassContribution(bodyB, cs.rB, dir);
            const float total = eA + eB;
            return (total > 1e-10f) ? (1.0f / total) : 0.0f;
        };
        
        cs.effMassNormal = buildEffMass(cs.normal);
        cs.effMassTangent1 = buildEffMass(cs.tangent1);
        cs.effMassTangent2 = buildEffMass(cs.tangent2);
        
        const float excess = std::max(0.0f, m.depth - PENETRATION_SLOP);
        cs.bias = (BAUMGARTE / dt) * excess;
        
        cs.restitution = std::sqrt(bodyA.properties.restitution * bodyB.properties.restitution);
        cs.friction = std::sqrt(bodyA.properties.friction * bodyB.properties.friction);
        
        cs.lambdaN = cs.lambdaT1 = cs.lambdaT2 = 0.0f;
        
        m_states.push_back(cs);
    }
}

void ContactSolver::solveIteration(float dt) {
    for (ContactState& cs : m_states) {
        RigidBody& bodyA = *cs.bodyA;
        RigidBody& bodyB = *cs.bodyB;

        // 1. NORMAL IMPULSE
        {
            const glm::vec3 vRel = relativeVelocity(bodyA, bodyB, cs.rA, cs.rB, dt);
            const float vNormal = glm::dot(vRel, cs.normal);
            const float e = (std::abs(vNormal) > RESTITUTION_VELOCITY_THRESHOLD) ? cs.restitution : 0.0f;
            
            const float deltaLambdaN = (-(1.0f + e) * vNormal + cs.bias) * cs.effMassNormal;
            
            const float lambdaN_old = cs.lambdaN;
            cs.lambdaN = std::max(0.0f, cs.lambdaN + deltaLambdaN);
            const float lambdaN_delta = cs.lambdaN - lambdaN_old;
            
            const glm::vec3 jN = lambdaN_delta * cs.normal;
            applyImpulse(bodyA, jN, cs.rA, dt);
            applyImpulse(bodyB, -jN, cs.rB, dt);
        }

        // 2. FRICTION IMPULSE
        {
            const float maxFriction = cs.friction * cs.lambdaN;
            if (maxFriction < 1e-8f) continue;
            
            const glm::vec3 vRel = relativeVelocity(bodyA, bodyB, cs.rA, cs.rB, dt);
            
            // Tangent 1
            {
                const float vT1 = glm::dot(vRel, cs.tangent1);
                const float deltaLambdaT1 = -vT1 * cs.effMassTangent1;
                const float lambdaT1_old = cs.lambdaT1;
                cs.lambdaT1 = glm::clamp(cs.lambdaT1 + deltaLambdaT1, -maxFriction, maxFriction);
                const float lambdaT1_delta = cs.lambdaT1 - lambdaT1_old;
                const glm::vec3 jT1 = lambdaT1_delta * cs.tangent1;
                applyImpulse(bodyA, jT1, cs.rA, dt);
                applyImpulse(bodyB, -jT1, cs.rB, dt);
            }
            
            // Tangent 2
            {
                const float vT2 = glm::dot(vRel, cs.tangent2);
                const float deltaLambdaT2 = -vT2 * cs.effMassTangent2;
                const float lambdaT2_old = cs.lambdaT2;
                cs.lambdaT2 = glm::clamp(cs.lambdaT2 + deltaLambdaT2, -maxFriction, maxFriction);
                const float lambdaT2_delta = cs.lambdaT2 - lambdaT2_old;
                const glm::vec3 jT2 = lambdaT2_delta * cs.tangent2;
                applyImpulse(bodyA, jT2, cs.rA, dt);
                applyImpulse(bodyB, -jT2, cs.rB, dt);
            }
        }
    }
}

void ContactSolver::solve(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt) {
    if (manifolds.empty()) return;
    preStep(bodies, manifolds, dt);
    for (int i = 0; i < ITERATIONS; i++) {
        solveIteration(dt);
    }
}
