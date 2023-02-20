#include <map>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>

#include <engine/Mesh.h>

using namespace Engine;
using namespace Engine::_internal;

static_assert(sizeof(MeshVertex) == (sizeof(float) * 9
                                     + MAX_WEIGHTS_PER_BONE * sizeof(unsigned int)
                                     + MAX_WEIGHTS_PER_BONE * sizeof(float)));

static inline glm::vec3 aiToGlmVec3(const aiVector3D &vec)
{
    return glm::vec3(vec.x, vec.y, vec.z);
}

static inline glm::mat4 aiToGlmMat4(const aiMatrix4x4 &mat)
{
    return glm::transpose(glm::make_mat4(&mat.a1));
}

static inline glm::quat aiToGlmQuat(const aiQuaternion &q)
{
    return glm::quat(q.w, q.x, q.y, q.z);
}

void Mesh::render()
{
    m_vertexBuffer.drawTriangles(m_indexBuffer);
}

bool Mesh::fromFile(const std::string &filename)
{
    const auto scene = m_importer.ReadFile(filename.c_str(),
                                           aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs |
                                           aiProcess_JoinIdenticalVertices);

    if (scene == nullptr)
    {
        spdlog::error("cannot load file {}: {}", filename, m_importer.GetErrorString());
        return false;
    }

    // read all the vertex and bone data from the file (pos, normals, texture coords, ...)
    // and fill the vertex buffer with that data
    std::vector<MeshVertex> vertices;
    fillBasicVertexData(vertices);
    fillBoneData(vertices);

    // set vertex buffer data layout
    m_vertexBuffer.addFloatLayoutAttribute(3);  // position
    m_vertexBuffer.addFloatLayoutAttribute(3);  // normals
    m_vertexBuffer.addFloatLayoutAttribute(3);  // texture coords
    m_vertexBuffer.addUnsignedIntLayoutAttribute(MAX_WEIGHTS_PER_BONE / 2);  // bone ids
    m_vertexBuffer.addUnsignedIntLayoutAttribute(MAX_WEIGHTS_PER_BONE / 2);  // bone ids
    m_vertexBuffer.addFloatLayoutAttribute(MAX_WEIGHTS_PER_BONE / 2);  // bone weights
    m_vertexBuffer.addFloatLayoutAttribute(MAX_WEIGHTS_PER_BONE / 2);  // bone weights

    // upload vertices to vertex buffer in GPU memory
    m_vertexBuffer.setData(vertices);
    return true;
}

std::optional<unsigned int> Mesh::getAnimationId(const std::string &animationName)
{
    auto scene = m_importer.GetScene();
    assert(scene != nullptr);

    for (unsigned int i = 0; i < scene->mNumAnimations; i++)
    {
        const auto animation = scene->mAnimations[i];
        if (animationName == animation->mName.C_Str())
        {
            return std::optional{i};
        }
    }

    return std::nullopt;
}

void
Mesh::getAnimationTransforms(unsigned int animationId, float normalizedTime, std::vector<glm::mat4> &boneTransforms)
{
    auto scene = m_importer.GetScene();
    assert(scene != nullptr);

    assert(animationId < scene->mNumAnimations);
    const auto animation = scene->mAnimations[animationId];

    if (boneTransforms.size() < getBoneCount())
    {
        boneTransforms.resize(getBoneCount());
    }

    buildBoneTransforms(animation, normalizedTime, scene->mRootNode, m_globalInverseTransform, boneTransforms);
}

void Mesh::fillBasicVertexData(std::vector<MeshVertex> &vertices)
{
    auto scene = m_importer.GetScene();
    assert(scene != nullptr);

    std::vector<unsigned int> indices;
    m_meshIndexOffsets.resize(scene->mNumMeshes);
    for (unsigned int i = 0; i < scene->mNumMeshes; i++)
    {
        const auto mesh = scene->mMeshes[i];
        const auto indexOffset = vertices.size();
        m_meshIndexOffsets[i] = indexOffset;
        vertices.resize(mesh->mNumVertices + indexOffset, MeshVertex{
                .position = glm::vec3(),
                .normals = glm::vec3(),
                .textureCoords = glm::vec3(),
                .boneIds = {},
                .boneWeights = {},
        });

        for (unsigned int v = 0; v < mesh->mNumVertices; v++)
        {
            auto &vertex = vertices[indexOffset + v];
            vertex.position = aiToGlmVec3(mesh->mVertices[v]);
            vertex.normals = aiToGlmVec3(mesh->mNormals[v]);
            vertex.textureCoords = mesh->HasTextureCoords(0)
                                   ? aiToGlmVec3(mesh->mTextureCoords[0][i])
                                   : glm::vec3();
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; f++)
        {
            const auto &face = mesh->mFaces[f];
            assert(face.mNumIndices == 3);
            indices.push_back(indexOffset + face.mIndices[0]);
            indices.push_back(indexOffset + face.mIndices[1]);
            indices.push_back(indexOffset + face.mIndices[2]);
        }
    }

    // upload indices to index buffer in GPU memory
    m_indexBuffer.setIndices(indices);

    spdlog::info("loaded {} meshes with {} vertices and {} indices in total",
                 scene->mNumMeshes, vertices.size(), indices.size());
}

void Mesh::fillBoneData(std::vector<_internal::MeshVertex> &vertices)
{
    auto scene = m_importer.GetScene();
    assert(scene != nullptr);

    m_globalInverseTransform = aiToGlmMat4(scene->mRootNode->mTransformation);
    m_globalInverseTransform = glm::inverse(m_globalInverseTransform);

    for (unsigned int i = 0; i < scene->mNumMeshes; i++)
    {
        const auto mesh = scene->mMeshes[i];
        unsigned int meshOffset = getMeshIndexOffset(i);
        if (mesh->HasBones())
        {
            for (unsigned int b = 0; b < mesh->mNumBones; b++)
            {
                const auto bone = mesh->mBones[b];
                for (unsigned int w = 0; w < bone->mNumWeights; w++)
                {
                    auto boneId = getBoneIdCreate(bone->mName.C_Str());
                    m_boneOffsetMatrixCache[boneId] = aiToGlmMat4(bone->mOffsetMatrix);

                    const auto &weight = bone->mWeights[w];
                    auto &vertex = vertices[meshOffset + weight.mVertexId];

                    bool assigned = false;
                    for (unsigned int n = 0; n < MAX_WEIGHTS_PER_BONE; n++)
                    {
                        if (vertex.boneWeights[n] != 0.0f)
                            continue;

                        vertex.boneIds[n] = boneId;
                        vertex.boneWeights[n] = weight.mWeight;
                        assigned = true;
                        break;
                    }

                    assert(assigned);
                }
            }
        }
    }
}

void Mesh::buildBoneTransforms(const aiAnimation *animation, float normalizedTime, const aiNode *node,
                               glm::mat4 parentTransform, std::vector<glm::mat4> &boneTransforms)
{
    // the nodeTransform matrix transforms from the node's coordinate system to its
    // parent node's coordinate system.
    auto nodeTransform = aiToGlmMat4(node->mTransformation);

    // if the node is not a dummy node but an actual bone, it has a set of keyframes
    // associated with it. their translation, scaling and rotation components can be used
    // to replace the nodeTransform. this is the part that makes the animated mesh move!
    auto keyframeSet = getBoneKeyframes(animation, node->mName.C_Str());
    if (keyframeSet != nullptr)
    {
        assert(keyframeSet->mNumPositionKeys == keyframeSet->mNumRotationKeys);
        assert(keyframeSet->mNumRotationKeys == keyframeSet->mNumScalingKeys);

        unsigned int t = (unsigned int) (normalizedTime * (float) keyframeSet->mNumRotationKeys);

        auto translation = aiToGlmVec3(keyframeSet->mPositionKeys[t].mValue);
        auto scaling = aiToGlmVec3(keyframeSet->mScalingKeys[t].mValue);
        auto rotation = aiToGlmQuat(keyframeSet->mRotationKeys[t].mValue);

        nodeTransform = glm::translate(translation) * glm::toMat4(rotation) * glm::scale(scaling);
    }

    // The parentTransform will transform a vertex from this node's parent's coordinate system all
    // the way to the scene-local coordinate system. By combining both the parentTransform and the
    // nodeTransform we extend this transform to the current node's coordinate system.
    auto globalTransform = parentTransform * nodeTransform;

    // If this node has an associated bone, we store its final transform in the vector of transforms
    // that is going to be uploaded to the vertex shader.
    auto boneId = getBoneId(node->mName.C_Str());
    if (boneId.has_value())
    {
        boneTransforms[boneId.value()] = /*m_globalInverseTransform **/
                globalTransform * getBoneOffsetMatrix(boneId.value());
    }

    // Now perform the same actions for the node's children as well.
    for (unsigned int i = 0; i < node->mNumChildren; i++)
    {
        buildBoneTransforms(animation, normalizedTime, node->mChildren[i],
                            globalTransform, boneTransforms);
    }
}

unsigned int Mesh::getBoneCount() const
{
    return m_boneOffsetMatrixCache.size();
}

const aiNodeAnim *Mesh::getBoneKeyframes(const aiAnimation *animation, const std::string &boneName)
{
    assert(animation != nullptr);
    for (unsigned int i = 0; i < animation->mNumChannels; i++)
    {
        const auto channel = animation->mChannels[i];
        if (boneName == channel->mNodeName.C_Str())
        {
            return channel;
        }
    }

    return nullptr;
}

unsigned int Mesh::getMeshIndexOffset(unsigned int meshIndex) const
{
    return m_meshIndexOffsets[meshIndex];
}

std::optional<unsigned int> Mesh::getBoneId(const std::string &boneName) const
{
    auto it = m_boneIds.find(boneName);
    if (it != m_boneIds.end())
    {
        return std::optional{it->second};
    }

    return std::nullopt;
}

unsigned int Mesh::getBoneIdCreate(const std::string &boneName)
{
    auto it = m_boneIds.find(boneName);
    if (it != m_boneIds.end())
    {
        return it->second;
    }

    auto id = m_boneOffsetMatrixCache.size();
    m_boneOffsetMatrixCache.resize(id + 1);
    m_boneIds.insert(std::make_pair(boneName, id));
    return id;
}

const glm::mat4 &Mesh::getBoneOffsetMatrix(unsigned int boneId) const
{
    assert(boneId < m_boneOffsetMatrixCache.size());
    return m_boneOffsetMatrixCache[boneId];
}
