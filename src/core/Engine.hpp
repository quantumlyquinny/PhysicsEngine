#pragma once
#include <SFML/Window.hpp>
#include <memory>

// Forward declarations — keep compile times fast
class PhysicsWorld;
class Renderer;
class Timer;

class Engine {
public:
    Engine(unsigned int width, unsigned int height, const char* title);
    ~Engine();

    // Non-copyable — owns unique GPU/physics state
    Engine(const Engine&)            = delete;
    Engine& operator=(const Engine&) = delete;

    void run();

private:
    void processInput();
    void update(float dt);
    void render();

    sf::Window          m_window;
    std::unique_ptr<PhysicsWorld> m_world;
    std::unique_ptr<Renderer>     m_renderer;
    std::unique_ptr<Timer>        m_timer;

    bool m_running = false;
};