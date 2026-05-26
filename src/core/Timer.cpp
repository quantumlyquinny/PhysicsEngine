#include "Timer.hpp"
#include <SFML/System/Clock.hpp>
#include <algorithm>

// A single static clock for the whole engine lifetime
static sf::Clock s_clock;

void Timer::tick() {
    const float now       = s_clock.getElapsedTime().asSeconds();
    m_deltaTime           = now - m_lastTime;
    m_lastTime            = now;

    // Clamp delta: if the window is dragged/minimised the OS can pause us
    // for seconds. Without this clamp, the accumulator explodes on resume.
    m_deltaTime = std::min(m_deltaTime, PHYSICS_STEP * MAX_STEPS_FRAME);

    m_accumulator += m_deltaTime;
    m_stepCount    = 0; // Reset per-frame step counter
}

bool Timer::shouldStep() const {
    return m_accumulator >= PHYSICS_STEP;
}

void Timer::consumeStep() {
    m_accumulator -= PHYSICS_STEP;
    ++m_stepCount;
}

float Timer::alpha() const {
    // Fraction of an unconsumed physics step remaining in the accumulator.
    // Renderer uses this to interpolate between prevState and currentState,
    // producing motion that looks smooth even at 30fps physics.
    return m_accumulator / PHYSICS_STEP;
}

float Timer::deltaTime() const {
    return m_deltaTime;
}