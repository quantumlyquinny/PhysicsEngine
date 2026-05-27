#pragma once
#include "../collision/Manifold.hpp"
#include "../RigidBody.hpp"
#include <vector>
#include <glm/glm.hpp>

// Sequential Impulse (SI) solver for rigid-body contacts.
class ContactSolver {
public:
    // Tuning constants
    static constexpr int ITERATIONS = 10;
    static constexpr float BAUMGARTE = 0.2f;
    static constexpr float PENETRATION_SLOP = 0.005f;
    static constexpr float RESTITUTION_VELOCITY_THRESHOLD = 0.5f;

    explicit ContactSolver(std::size_t maxContacts = 4096 * 4);
    
    // Entry point. Called by PhysicsWorld::solveConstraints()
    void solve(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt);

private:
    // Per-contact working state computed once in preStep()
    struct ContactState {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
        glm::vec3 normal;
        glm::vec3 tangent1;
        glm::vec3 tangent2;
        glm::vec3 rA;
        glm::vec3 rB;
        
        float effMassNormal = 0.0f;
        float effMassTangent1 = 0.0f;
        float effMassTangent2 = 0.0f;
        float bias = 0.0f;
        float restitution = 0.0f;
        float friction = 0.0f;
        
        // Accumulated impulses
        float lambdaN = 0.0f;
        float lambdaT1 = 0.0f;
        float lambdaT2 = 0.0f;
    };

    std::vector<ContactState> m_states;

    void preStep(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt);
    void solveIteration(float dt);

    static float effectiveMassContribution(const RigidBody& body, const glm::vec3& arm, const glm::vec3& dir);
    static glm::vec3 pointVelocity(const RigidBody& body, const glm::vec3& arm, float dt);
    static glm::vec3 relativeVelocity(const RigidBody& bodyA, const RigidBody& bodyB, const glm::vec3& rA, const glm::vec3& rB, float dt);
    static void applyImpulse(RigidBody& body, const glm::vec3& impulse, const glm::vec3& arm, float dt);
    static void computeTangentBasis(const glm::vec3& n, glm::vec3& t1, glm::vec3& t2);
    static void wake(RigidBody& body);
};