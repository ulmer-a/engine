#pragma once

#include <glm/glm.hpp>

namespace Engine {
    namespace Renderer {

        class Camera
        {
        public:
            Camera(glm::vec3 cameraPosition,
                   glm::vec3 lookAtPoint = glm::vec3(0.0, 0.0, 0.0),
                   glm::vec3 upVector = glm::vec3(0.0, 1.0, 0.0)
            );

            void setPosition(glm::vec3 cameraPosition);

            void setLookAtPoint(glm::vec3 lookAtPoint);

            void setLookAtVector(glm::vec3 lookAtVector);

            void setUpVector(glm::vec3 upVector);

            inline const glm::mat4 &getViewMatrix() const
            {
                return m_viewMatrix;
            }

        private:
            void updateViewMatrix();

        private:
            glm::mat4 m_viewMatrix;
            glm::vec3 m_cameraPosition, m_lookAtPoint, m_upVector;
        };
    } // Engine
} // Renderer
