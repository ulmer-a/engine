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

            void compileAndLinkShaders(const std::string &vertexShaderFile,
                                       const std::string &fragmentShaderFile);

            bool link();

            void use();

            void addShader(const Shader &shader);

            unsigned int getUniformByName(const char *name);

            void setUniform(unsigned int id, const glm::mat4 &mat);

            void setUniform(unsigned int id, const std::vector<glm::mat4> &mats);

        private:
            unsigned int m_id;
        };
    } // Engine
} // Renderer
