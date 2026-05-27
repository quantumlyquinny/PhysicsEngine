#pragma once
#include <vector>
#include "../physics/RigidBody.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Renderer {
public:
    Renderer(unsigned int width, unsigned int height);
    ~Renderer();

    void onResize(unsigned int width, unsigned int height);
    void beginFrame();
    void drawBodies(const std::vector<RigidBody>& bodies);
    void endFrame();

private:
    unsigned int m_shaderProgram;
    unsigned int m_cubeVAO, m_cubeVBO;
    glm::mat4 m_projection;
    glm::mat4 m_view;

    void initShader();
    void initCube();
};