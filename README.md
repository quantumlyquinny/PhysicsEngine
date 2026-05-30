# Custom 3D Physics Engine

![C++](https://img.shields.io/badge/C++-17-blue.svg)
![OpenGL](https://img.shields.io/badge/OpenGL-4.1_Core-brightgreen.svg)
![CMake](https://img.shields.io/badge/CMake-Build-orange.svg)
![License](https://img.shields.io/badge/License-MIT-purple.svg)

A from-scratch rigid-body physics engine built in modern C++17, capable of simulating
200–500 concurrent physics objects at real-time frame rates. This project bypasses
off-the-shelf libraries (PhysX, Bullet) to implement foundational collision detection
and dynamics algorithms by hand, paired with a custom OpenGL rendering pipeline.

<div align="center">
  <img src="docs/simulation_demo.gif" alt="Physics Simulation Demo" width="600"/>
</div>

## Why I Built This

Most game engines abstract away the physics layer entirely. I wanted to understand what
happens underneath — how GJK actually finds the closest features between two meshes,
why sequential impulse solvers converge, and what makes a physics loop numerically stable.
This project is the answer to those questions.

## Core Architecture

### Collision Pipeline
- **Broad Phase:** Octree spatial partitioning reduces collision pair candidates from
  O(n²) to O(n log n), enabling real-time performance at scale.
- **Narrow Phase:** GJK (Gilbert-Johnson-Keerthi) algorithm for boolean intersection
  testing, extended with EPA (Expanding Polytope Algorithm) for precise penetration
  depth and contact manifold generation.
- **Resolution:** Sequential Impulse solver handles resting contacts, friction
  coefficients, and restitution (bounciness) per material.

### Physics Integration
- Full rigid-body dynamics: velocity, acceleration, mass, and inertia tensor integration
  via Verlet Integration for high-accuracy particle dynamics.
- Fixed time-step physics loop **decoupled from the render loop**, with quaternion/vector
  interpolation (alpha blending) to produce smooth visuals at any frame rate.

### Rendering
- Lightweight custom OpenGL 4.1 debug renderer — no game engine overhead.
- Designed purely to visualise physics state: wireframes, contact normals, AABB overlays.

## Tech Stack

| Layer | Technology |
|---|---|
| Language | C++17 |
| Build System | CMake 3.10+ |
| Graphics API | OpenGL 4.1 Core Profile |
| Extension Loader | GLAD |
| Windowing & Input | SFML |
| Mathematics | GLM |

## Build Instructions

### Prerequisites
- C++17 compiler (GCC, Clang, or MSVC)
- CMake v3.10+
- SFML, GLAD, and GLM accessible to your compiler

### Compile

```bash
git clone https://github.com/quantumlyquinny/3d-physics-engine
cd 3d-physics-engine
mkdir build && cd build
cmake ..
make
./PhysicsEngine
```

## What I Learned

Implementing GJK from scratch was the hardest part — the algorithm is elegant but the
edge cases (degenerate simplices, parallel faces) are brutal. The EPA extension took
longer than the rest of the project combined. If you're studying collision detection,
start with [Dyn4j's excellent breakdown](https://dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/).
