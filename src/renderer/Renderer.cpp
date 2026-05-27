#include "Renderer.hpp"
#include <glad/glad.h>
#include <glm/gtx/quaternion.hpp>
#include <iostream>

// --- Basic Hardcoded Shaders ---
const char* vertexShaderSource = R"(
#version 410 core
layout (location = 0) in vec3 aPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 410 core
out vec4 FragColor;
uniform vec3 objectColor;
void main() {
    float ambient = 0.5;
    FragColor = vec4(objectColor * ambient, 1.0);
}
)";

Renderer::Renderer(unsigned int width, unsigned int height) {
    initShader();
    initCube();
    
    m_view = glm::lookAt(glm::vec3(0.0f, 10.0f, 25.0f), 
                         glm::vec3(0.0f, 2.0f, 0.0f), 
                         glm::vec3(0.0f, 1.0f, 0.0f));
    onResize(width, height);
}

Renderer::~Renderer() {
    glDeleteVertexArrays(1, &m_cubeVAO);
    glDeleteBuffers(1, &m_cubeVBO);
    glDeleteProgram(m_shaderProgram);
}

void Renderer::onResize(unsigned int width, unsigned int height) {
    m_projection = glm::perspective(glm::radians(45.0f), (float)width / (float)height, 0.1f, 100.0f);
}

void Renderer::beginFrame() {
    glUseProgram(m_shaderProgram);
    
    int viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    int projLoc = glGetUniformLocation(m_shaderProgram, "projection");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projection));
}

void Renderer::drawBodies(const std::vector<RigidBody>& bodies) {
    glBindVertexArray(m_cubeVAO);
    int modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    int colorLoc = glGetUniformLocation(m_shaderProgram, "objectColor");

    for (const auto& body : bodies) {
        if (body.id == INVALID_BODY_ID) continue;

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, body.state.renderPosition);
        model *= glm::toMat4(body.state.renderOrientation);
        
        // Explicitly scale the floor to be flat, and the dynamic cubes to be 1x1x1
        glm::vec3 scale = body.isStatic ? glm::vec3(20.0f, 2.0f, 20.0f) : glm::vec3(1.0f, 1.0f, 1.0f);
        model = glm::scale(model, scale);

        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

        glm::vec3 color = body.isStatic ? glm::vec3(0.5f, 0.5f, 0.5f) : glm::vec3(1.0f, 0.5f, 0.2f);
        glUniform3fv(colorLoc, 1, glm::value_ptr(color));

        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    glBindVertexArray(0);
}

void Renderer::endFrame() {
    glUseProgram(0);
}

void Renderer::initShader() {
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Renderer::initCube() {
    float vertices[] = {
        -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f, -0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,  0.5f,
        -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f, -0.5f,
        -0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f, -0.5f
    };

    glGenVertexArrays(1, &m_cubeVAO);
    glGenBuffers(1, &m_cubeVBO);

    glBindVertexArray(m_cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}