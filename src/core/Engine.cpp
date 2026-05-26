#include "Engine.hpp"
#include "Timer.hpp"
#include "../physics/PhysicsWorld.hpp"
#include "../renderer/Renderer.hpp"

// GLAD must be included before any OpenGL header
#include <glad/glad.h>
#include <SFML/Window.hpp>
#include <stdexcept>

Engine::Engine(unsigned int width, unsigned int height, const char* title)
{
    // --- SFML Window with OpenGL 4.1 Core Profile ---
    sf::ContextSettings settings;
    settings.depthBits         = 24;
    settings.stencilBits       = 8;
    settings.antialiasingLevel = 4;
    settings.majorVersion      = 4;
    settings.minorVersion      = 1;
    settings.attributeFlags    = sf::ContextSettings::Core;

    m_window.create(
        sf::VideoMode(width, height), title,
        sf::Style::Default, settings
    );
    m_window.setVerticalSyncEnabled(false); // We control timing ourselves
    m_window.setMouseCursorGrabbed(true);

    // --- Bootstrap GLAD (must happen after a valid GL context exists) ---
    if (!gladLoadGL()) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glViewport(0, 0, static_cast<int>(width), static_cast<int>(height));
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // --- Allocate systems (pre-allocate their internal pools here) ---
    m_world    = std::make_unique<PhysicsWorld>();
    //m_renderer = std::make_unique<Renderer>(width, height);
    m_timer    = std::make_unique<Timer>();

    m_world->setGravity(glm::vec3(0.0f, -9.81f, 0.0f));
    m_running = true;
}

Engine::~Engine() {
    // Unique_ptrs clean up in reverse order automatically.
    // GPU resources are released by ~Renderer().
    m_window.close();
}

// ─────────────────────────────────────────────────────────────────────────────
// THE CORE LOOP — Fixed Physics, Unlocked Rendering, Interpolated Transforms
// ─────────────────────────────────────────────────────────────────────────────
void Engine::run() {
    while (m_running) {
        // 1. MEASURE — tick the clock, compute delta, fill accumulator
        m_timer->tick();

        // 2. INPUT — SFML event pump (always first, before any state mutation)
        processInput();

        // 3. PHYSICS — consume the accumulator in fixed-size bites
        //    MAX_STEPS_FRAME prevents the spiral of death on slow frames
        int steps = 0;
        while (m_timer->shouldStep() && steps < Timer::MAX_STEPS_FRAME) {
            m_world->step(Timer::PHYSICS_STEP);
            m_timer->consumeStep();
            ++steps;
        }

        // 4. INTERPOLATE — blend previous/current state for smooth rendering
        //    alpha = 0 → fully at previous state; alpha = 1 → fully at current
        const float alpha = m_timer->alpha();
        for (auto& body : m_world->getBodies()) {
            // Write ONLY to renderPosition/renderOrientation — never to physics state
            const_cast<RigidBody&>(body).state.renderPosition =
                glm::mix(body.state.prevPosition, body.state.position, alpha);
            const_cast<RigidBody&>(body).state.renderOrientation =
                glm::slerp(body.state.prevOrientation, body.state.orientation, alpha);
        }

        // 5. RENDER — draw interpolated state; physics is not touched
        render();
    }
}

void Engine::processInput() {
    sf::Event event;
    while (m_window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            m_running = false;
        if (event.type == sf::Event::KeyPressed &&
            event.key.code == sf::Keyboard::Escape)
            m_running = false;
        if (event.type == sf::Event::Resized) {
            glViewport(0, 0, event.size.width, event.size.height);
//          m_renderer->onResize(event.size.width, event.size.height);
        }
        // Forward to camera controller, debug UI, etc. in later steps
    }
}

void Engine::update(float dt) {
    // Intentionally thin — PhysicsWorld::step() IS the update.
    // This hook is for non-physics game logic (camera, UI state, etc.)
    (void)dt;
}

void Engine::render() {
    glClearColor(0.08f, 0.08f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //m_renderer->beginFrame();
    //m_renderer->drawBodies(m_world->getBodies());
    //m_renderer->drawDebugOverlay(); // AABB wireframes, contact normals
    //m_renderer->endFrame();

    m_window.display();
}