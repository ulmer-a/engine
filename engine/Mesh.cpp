#include <glm/glm.hpp>
#include <spdlog/spdlog.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>

#include <engine/Mesh.h>

using namespace Engine;

struct MeshVertex
{
    glm::vec3 position;
    glm::vec3 normals;
};

static_assert(sizeof(MeshVertex) == sizeof(float) * 6);

bool Mesh::fromFile(const std::string &filename)
{
    Assimp::Importer Importer;
    const auto scene = Importer.ReadFile(filename.c_str(),
                                         aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs |
                                         aiProcess_JoinIdenticalVertices);

    if (scene == nullptr)
        return false;

    auto aiToGlmVec3 = [](const aiVector3D &vec) {
        return glm::vec3(vec.x, vec.y, vec.z);
    };

    std::vector<MeshVertex> vertices;
    std::vector<unsigned int> indices;
    for (unsigned int i = 0; i < scene->mNumMeshes; i++)
    {
        const auto mesh = scene->mMeshes[i];
        const auto prevVertexCount = vertices.size();
        vertices.resize(mesh->mNumVertices + prevVertexCount);

        for (unsigned int v = 0; v < mesh->mNumVertices; v++)
        {
            vertices[prevVertexCount + v] = MeshVertex{
                    aiToGlmVec3(mesh->mVertices[v]),
                    aiToGlmVec3(mesh->mNormals[v]),
                    // mesh->HasTextureCoords(0) ? aiToGlmVec3(mesh->mTextureCoords[0][i]) : glm::vec3(),
            };
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; f++)
        {
            const auto &face = mesh->mFaces[f];
            assert(face.mNumIndices == 3);
            indices.push_back(prevVertexCount + face.mIndices[0]);
            indices.push_back(prevVertexCount + face.mIndices[1]);
            indices.push_back(prevVertexCount + face.mIndices[2]);
        }
    }

    m_vertexBuffer.addFloatLayoutAttribute(3);  // position
    m_vertexBuffer.addFloatLayoutAttribute(3);  // normals
    m_vertexBuffer.setData(vertices);
    m_indexBuffer.setIndices(indices);

    spdlog::debug("loaded {} meshes with {} vertices and {} indices in total",
                  scene->mNumMeshes, vertices.size(), indices.size());
    return true;
}

void Mesh::render()
{
    m_vertexBuffer.drawTriangles(m_indexBuffer);
}
