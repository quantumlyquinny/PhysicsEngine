#pragma once
#include "RigidBody.hpp"
#include "collision/IConvexShape.hpp"
#include <vector>
#include <memory>
#include <glm/glm.hpp>

// Forward declarations
class Integrator;
class BroadPhase;
class NarrowPhase;
class ContactSolver; // <--- ADDED HERE
struct Manifold;

class PhysicsWorld {
public:
    static constexpr std::size_t MAX_BODIES = 4096;

    PhysicsWorld();
    ~PhysicsWorld();

    BodyID createBody(const RigidBodyState& state, 
                      const RigidBodyProperties& props, 
                      bool isStatic = false);
    void destroyBody(BodyID id);
    RigidBody* getBody(BodyID id);
    
    const std::vector<RigidBody>& getBodies() const { return m_bodies; }
    
    void step(float dt);
    void setCollisionShape(BodyID id, std::shared_ptr<IConvexShape> shape);
    void setGravity(const glm::vec3& gravity);
    
private:
    std::vector<RigidBody>  m_bodies;
    std::vector<BodyID>     m_freeList;
    glm::vec3 m_gravity = glm::vec3(0.0f, -9.81f, 0.0f);

    std::unique_ptr<Integrator>    m_integrator;
    std::unique_ptr<BroadPhase>    m_broadPhase;
    std::unique_ptr<NarrowPhase>   m_narrowPhase;
    
    // The Contact Solver is now safely inside the class!
    std::unique_ptr<ContactSolver> m_contactSolver; 

    void solveConstraints(const std::vector<Manifold>& manifolds, float dt);
    void updateSleepState(float dt);
};