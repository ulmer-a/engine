#pragma once

#include <map>
#include <unordered_set>
#include <string>
#include <assimp/Importer.hpp>
#include <engine/IndexBuffer.h>
#include <engine/VertexBuffer.h>

struct aiScene;
struct aiNode;
struct aiBone;
struct aiAnimation;
struct aiNodeAnim;

namespace Engine {

    namespace _internal {
        const size_t MAX_WEIGHTS_PER_BONE = 10;

        struct MeshVertex
        {
            glm::vec3 position;
            glm::vec3 normals;
            glm::vec3 textureCoords;
            unsigned int boneIds[MAX_WEIGHTS_PER_BONE];
            float boneWeights[MAX_WEIGHTS_PER_BONE];
        };
    }

    class Mesh
    {
    public:
        Mesh() = default;

        ~Mesh() = default;

        void render();

        bool fromFile(const std::string &filename);

        std::optional<unsigned int> getAnimationId(const std::string &animationName);

        void
        getAnimationTransforms(unsigned int animationId, float normalizedTime, std::vector<glm::mat4> &boneTransforms);

    private:
        void fillBasicVertexData(std::vector<_internal::MeshVertex> &vertices);

        void fillBoneData(std::vector<_internal::MeshVertex> &vertices);

        void buildBoneTransforms(const aiAnimation *animation, float normalizedTime, const aiNode *node,
                                 glm::mat4 parentTransform, std::vector<glm::mat4> &boneTransforms);

        /**
         * this function returns the number of bones across all meshes
         *
         * @return the number of bones
         */
        unsigned int getBoneCount() const;

        /**
         * this function returns a pointer to a aiNodeAnim structure corresponding
         * to animation for the specified bone.
         *
         * @param boneName
         * @return
         */
        const aiNodeAnim *getBoneKeyframes(const aiAnimation *animation, const std::string &boneName);

        /**
         * this function returns the index of the mesh's first vertex in the vertex buffer.
         *
         * @param meshIndex index of the mesh as it occurs in aiScene->mMeshes.         *
         * @return the index of the mesh's first vertex in the vertex buffer.
         */
        unsigned int getMeshIndexOffset(unsigned int meshIndex) const;

        /**
         * this function translates the name of a bone into it's id. If boneName is
         * the name of a node, an id is only returned if the node corresponds to a bone.
         *
         * @param boneName the name of the node or the bone
         * @return the unique id of the bone
         */
        std::optional<unsigned int> getBoneId(const std::string &boneName) const;

        /**
         * this function translates the name of a bone into it's id. if the bone is not
         * yet contained in the list, this function will allocate a new id an return it.
         * do not call this function with node names!
         *
         * @param boneName the name of the bone
         * @return the unique id of the bone
         */
        unsigned int getBoneIdCreate(const std::string &boneName);

        /**
         * this function returns the offset matrix of a given bone that can be
         * used to transform a vertex from the scene's coordinate system into
         * the bone's coordinate system.
         *
         * @param boneId bone id
         * @return 4x4 offset matrix
         */
        const glm::mat4 &getBoneOffsetMatrix(unsigned int boneId) const;

    private:
        /**
         * Assimp's importer object holds the imported file's contents in memory.
         */
        Assimp::Importer m_importer;

        /**
         * bone information cache
         */
        std::map<std::string, unsigned int> m_boneIds;
        std::vector<glm::mat4> m_boneOffsetMatrixCache;

        /**
         * all vertices of all meshes are contained in a single vertex buffer. this
         * vector holds the first vertex's index of a given mesh in the vertex buffer.
         */
        std::vector<unsigned int> m_meshIndexOffsets;

        /**
         * vertex and index buffers refer to the vertex data on the GPU and
         * are used to render them on the screen.
         */
        Renderer::IndexBuffer m_indexBuffer;
        Renderer::VertexBuffer m_vertexBuffer;
    };
}
