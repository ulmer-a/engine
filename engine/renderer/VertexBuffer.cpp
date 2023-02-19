#include <GL/glew.h>
#include <engine/IndexBuffer.h>
#include <engine/VertexBuffer.h>

using namespace Engine::Renderer;

VertexBuffer::VertexBuffer()
{
    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_vertexBuffer);

    bind();
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    unbind();
}

VertexBuffer::~VertexBuffer()
{
    glDeleteBuffers(1, &m_vertexBuffer);
    glDeleteVertexArrays(1, &m_vertexArray);
}

void VertexBuffer::drawTriangles()
{
    bind();
    assert(m_vertexCount > 0);
    assert(m_vertexCount % 3 == 0);
    glDrawArrays(GL_TRIANGLES, 0, m_vertexCount);
    unbind();
}

void VertexBuffer::drawTriangles(IndexBuffer &indexBuffer)
{
    bind();
    indexBuffer.bind();
    assert(indexBuffer.indexCount() > 0);
    assert(indexBuffer.indexCount() % 3 == 0);
    glDrawElements(GL_TRIANGLES, indexBuffer.indexCount(), GL_UNSIGNED_INT, nullptr);
    IndexBuffer::unbind();
    unbind();
}

void VertexBuffer::bind()
{
    glBindVertexArray(m_vertexArray);
}

void VertexBuffer::unbind()
{
    glBindVertexArray(0);
}

void VertexBuffer::setRawData(const void *data, size_t size, size_t count)
{
    assert(m_attributes.size() > 0);
    m_vertexCount = count;

    bind();
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, size * count, data, GL_STATIC_DRAW);
    setVertexAttributes(size);
    unbind();
}

void VertexBuffer::addFloatLayoutAttribute(size_t count)
{
    m_attributes.push_back(VertexAttribute{
            GL_FLOAT,
            sizeof(float),
            count,
    });
}

void VertexBuffer::addUnsignedIntLayoutAttribute(size_t count)
{
    m_attributes.push_back(VertexAttribute{
            GL_UNSIGNED_INT,
            sizeof(unsigned int),
            count
    });
}

void VertexBuffer::setVertexAttributes(size_t vertexSize)
{
    // assuming the vertex buffer object is already bound!

    size_t offset = 0;
    for (unsigned int i = 0; i < m_attributes.size(); i++)
    {
        const auto &attr = m_attributes[i];
        if (attr.type == GL_UNSIGNED_INT) // TODO catch all integer types
        {
            glVertexAttribIPointer(i, attr.count, attr.type, vertexSize, (void *) offset);
        } else
        {
            glVertexAttribPointer(i, attr.count, attr.type, GL_FALSE, vertexSize, (void *) offset);
        }
        glEnableVertexAttribArray(i);
        offset += attr.size * attr.count;
    }

    m_attributes.clear();
}
