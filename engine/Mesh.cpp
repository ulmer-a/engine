#include <map>
#include <glm/glm.hpp>
#include <spdlog/spdlog.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>

#include <engine/Mesh.h>

using namespace Engine;
using namespace Engine::_internal;

static inline glm::vec3 aiToGlmVec3(const aiVector3D &vec)
{
    return glm::vec3(vec.x, vec.y, vec.z);
}

bool Mesh::read(const std::string &filename, bool rigged)
{
    Assimp::Importer importer;
    const auto scene = importer.ReadFile(filename.c_str(),
                                         aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs |
                                         aiProcess_JoinIdenticalVertices);

    if (scene == nullptr)
    {
        spdlog::error("cannot load file {}: {}", filename, importer.GetErrorString());
        return false;
    }

    // read all the vertex data from the file (pos, normals, texture coords, ...)
    std::vector<MeshVertex> vertices;
    readMeshVertices(scene, vertices);

    if (rigged)
    {
        fillVertexBoneInfo(scene, vertices);
    }

    m_vertexBuffer.addFloatLayoutAttribute(3);  // position
    m_vertexBuffer.addFloatLayoutAttribute(3);  // normals
    m_vertexBuffer.addFloatLayoutAttribute(3);  // texture coords
    for (unsigned i = 0; i < MAX_WEIGHTS_PER_BONE; i++)
    {
        m_vertexBuffer.addUnsignedIntLayoutAttribute(1);    // bone id
        m_vertexBuffer.addFloatLayoutAttribute(1);          // weight
    }

    m_vertexBuffer.setData(vertices);
    return true;
}

void Mesh::readMeshVertices(const aiScene *scene, std::vector<MeshVertex> &vertices)
{
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
                .boneWeights = {IdWeightPair{(unsigned int) -1, 0.0f}}
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
    m_indexBuffer.setIndices(indices);

    spdlog::debug("loaded {} meshes with {} vertices and {} indices in total",
                  scene->mNumMeshes, vertices.size(), indices.size());
}

void Mesh::fillVertexBoneInfo(const aiScene *scene, std::vector<MeshVertex> &vertices)
{
    for (unsigned int i = 0; i < scene->mNumMeshes; i++)
    {
        const auto mesh = scene->mMeshes[i];
        unsigned int meshOffset = m_meshIndexOffsets[i];
        if (mesh->HasBones())
        {
            for (unsigned int b = 0; b < mesh->mNumBones; b++)
            {
                const auto bone = mesh->mBones[b];
                for (unsigned int w = 0; w < bone->mNumWeights; w++)
                {
                    const auto weight = bone->mWeights;
                    auto &vertex = vertices[meshOffset + weight->mVertexId];

                    bool assigned = false;
                    for (unsigned int n = 0; n < MAX_WEIGHTS_PER_BONE; n++)
                    {
                        if (vertex.boneWeights[n].boneId > (unsigned int) -1)
                            continue;

                        vertex.boneWeights[n].boneId = b; // TODO !!!!!
                        vertex.boneWeights[n].weight = weight->mWeight;
                        assigned = true;
                    }

                    assert(assigned);
                }
            }
        }
    }
}


void Mesh::render()
{
    m_vertexBuffer.drawTriangles(m_indexBuffer);
}

//const aiNode *Mesh::findNode(const std::string &nodeName, const aiNode *node)
//{
//    if (node == nullptr)
//        return findNode(nodeName, m_scene->mRootNode);
//}
//
//void Mesh::markNodeUsed(const std::string &nodeName, unsigned int meshIndex)
//{
//    const auto node = findNode(nodeName);
//    assert(node != nullptr);
//
//    if (m_nodesUsed.find(nodeName) == m_nodesUsed.end())
//    {
//        m_nodesUsed.insert(nodeName);
//
//        const auto parentNode = node->mParent;
//        if (!nodeContainsMesh(node, meshIndex) && parentNode != nullptr)
//        {
//            markNodeUsed(parentNode->mName.C_Str(), meshIndex);
//        }
//    }
//}
//
//bool Mesh::nodeContainsMesh(const aiNode *node, unsigned int meshIndex)
//{
//    assert(node != nullptr);
//    for (unsigned int i = 0; i < node->mNumMeshes; i++)
//    {
//        if (meshIndex == node->mMeshes[i])
//            return true;
//    }
//    return false;
//}
