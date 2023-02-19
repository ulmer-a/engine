#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace Engine {
    namespace Renderer {

        class IndexBuffer;

        /**
         * The VertexBuffer class controls not only an actual GPU
         * vertex buffer but also an index buffer.
         */
        class VertexBuffer
        {
        public:
            explicit VertexBuffer();

            VertexBuffer(const VertexBuffer &) = delete;

            ~VertexBuffer();

            /**
             * drawTriangles() binds the vertex buffer and draws
             * its contents as triangles.
             */
            void drawTriangles();

            /**
             * drawTriangles() draws the vertices selected by the indices
             * from the index buffer as triangles.
             */
            void drawTriangles(IndexBuffer &indexBuffer);

            /**
             * Submit data contained in the std::vector to GPU memory.
             * @tparam T data type of objects contained in the vector.
             * @param vertices the vector containing the data to be submitted.
             */
            template<typename T>
            void setData(const std::vector<T> &vertices)
            {
                setRawData((const void *) vertices.data(), sizeof(T), vertices.size());
            }

            /**
             * Describe the layout of your vertices by calling this function.
             * @param count Number of floats the current attribute contains.
             */
            void addFloatLayoutAttribute(size_t count);

            void addUnsignedIntLayoutAttribute(size_t count);

        private:
            void setRawData(const void *data, size_t size, size_t count);

            void setVertexAttributes(size_t vertexSize);

            void bind();

            static void unbind();

        private:
            struct VertexAttribute
            {
                int type;
                size_t size;
                size_t count;
            };

            unsigned int m_vertexCount;
            std::vector<VertexAttribute> m_attributes;
            unsigned int m_vertexArray, m_vertexBuffer;
        };
    } // Engine
} // Renderer
