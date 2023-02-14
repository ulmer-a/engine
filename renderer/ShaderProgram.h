//
// Created by alex on 2/15/23.
//

#include <glm/glm.hpp>

#pragma once

namespace Engine {
    namespace Renderer {

        class Shader;

        class ShaderProgram
        {
        public:
            explicit ShaderProgram();

            ShaderProgram(const ShaderProgram &) = delete;

            ~ShaderProgram();

            bool link();

            void bind();

            void addShader(const Shader &shader);

            unsigned int getUniformByName(const char *name);

            void setUniform(unsigned int id, const glm::mat4 &mat);

        private:
            unsigned int m_id;
        };

    } // Engine
} // Renderer
