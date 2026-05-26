#pragma once
#include "RigidBody.hpp"
#include <vector>
#include <memory>

// Forward declarations
class Integrator;
class BroadPhase;
class NarrowPhase;
class Manifold;

class PhysicsWorld {
public:
    static constexpr std::size_t MAX_BODIES = 4096; // Pre-allocation target

    PhysicsWorld();
    ~PhysicsWorld();

    BodyID createBody(const RigidBodyState& state,
                      const RigidBodyProperties& props,
                      bool isStatic = false);
    void   destroyBody(BodyID id); // Marks as free, no deallocation
    RigidBody* getBody(BodyID id); // Returns nullptr if invalid

    void step(float dt); // Called by Engine at PHYSICS_STEP rate

    void setGravity(const glm::vec3& g) { m_gravity = g; }
    const glm::vec3& getGravity() const  { return m_gravity; }

    // Read-only view for renderer (interpolated transforms only)
    const std::vector<RigidBody>& getBodies() const { return m_bodies; }

private:
    // --- Pre-allocated storage (zero per-frame heap activity) ---
    std::vector<RigidBody>  m_bodies;     // Indexed by BodyID
    std::vector<BodyID>     m_freeList;   // Recycled slots

    glm::vec3 m_gravity = glm::vec3(0.0f, -9.81f, 0.0f);

    // --- Systems (owned, initialized once) ---
    std::unique_ptr<Integrator>  m_integrator;
    std::unique_ptr<BroadPhase>  m_broadPhase;
    std::unique_ptr<NarrowPhase> m_narrowPhase;

    void solveConstraints(const std::vector<Manifold>& manifolds, float dt);
    void updateSleepState(float dt);
};