#pragma once

class Timer {
public:
    // PHYSICS_STEP is fixed — Verlet integration is step-size sensitive.
    // 120 Hz gives stability headroom; never let this be variable.
    static constexpr float PHYSICS_STEP    = 1.0f / 120.0f;
    static constexpr int   MAX_STEPS_FRAME = 8; // Spiral-of-death guard

    void   tick();             // Call once per frame
    bool   shouldStep() const; // True while accumulator >= PHYSICS_STEP
    void   consumeStep();      // Drain one physics step from accumulator
    float  alpha() const;      // Interpolation factor [0,1] for rendering
    float  deltaTime() const;  // Raw frame delta (for debug/display only)

private:
    float m_accumulator  = 0.0f;
    float m_lastTime     = 0.0f;
    float m_deltaTime    = 0.0f;
    int   m_stepCount    = 0;
};