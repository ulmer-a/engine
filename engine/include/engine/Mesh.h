#pragma once

#include <map>
#include <unordered_set>
#include <string>
#include <engine/IndexBuffer.h>
#include <engine/VertexBuffer.h>

struct aiScene;

namespace Engine {

    namespace _internal {
        const size_t MAX_WEIGHTS_PER_BONE = 4;

        struct MeshVertex
        {
            glm::vec3 position;
            glm::vec3 normals;
            glm::vec3 textureCoords;
            unsigned int boneIds[4];
            float boneWeights[4];
        };

        static_assert(sizeof(MeshVertex) == (sizeof(float) * 9
                                             + MAX_WEIGHTS_PER_BONE * sizeof(unsigned int)
                                             + MAX_WEIGHTS_PER_BONE * sizeof(float)));
    }

    class Mesh
    {
    public:
        Mesh() = default;

        ~Mesh() = default;

        bool read(const std::string &filename, bool rigged = false);

        void render();

    private:
        void readMeshVertices(const aiScene *scene, std::vector<_internal::MeshVertex> &vertices);

        void fillVertexBoneInfo(const aiScene *scene, std::vector<_internal::MeshVertex> &vertices);

        void fillAnimationInfo(const aiScene *scene);

    private:
        std::vector<unsigned int> m_meshIndexOffsets;
        Renderer::IndexBuffer m_indexBuffer;
        Renderer::VertexBuffer m_vertexBuffer;
    };
}
