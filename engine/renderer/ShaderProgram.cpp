#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <spdlog/spdlog.h>
#include <engine/Shader.h>
#include <engine/ShaderProgram.h>

using namespace Engine::Renderer;

ShaderProgram::ShaderProgram()
        : m_id(glCreateProgram())
{
}

ShaderProgram::~ShaderProgram()
{
    glDeleteProgram(m_id);
}

bool ShaderProgram::link()
{
    glLinkProgram(m_id);

    int success;
    char messageBuffer[512];
    glGetProgramiv(m_id, GL_LINK_STATUS, &success);

    if (!success)
    {
        glGetProgramInfoLog(m_id, sizeof(messageBuffer), nullptr, messageBuffer);
        spdlog::error("OpenGL: error linking shader: {}", messageBuffer);
        return false;
    }

    return true;
}

void ShaderProgram::use()
{
    glUseProgram(m_id);
}

void ShaderProgram::addShader(const Shader &shader)
{
    glAttachShader(m_id, shader.getId());
}

unsigned int ShaderProgram::getUniformByName(const char *name)
{
    return glGetUniformLocation(m_id, name);
}

void ShaderProgram::setUniform(unsigned int id, float f)
{
    this->use();
    glUniform1f(id, f);
}

void ShaderProgram::setUniform(unsigned int id, const glm::mat4 &mat)
{
    this->use();
    glUniformMatrix4fv(id, 1, GL_FALSE, glm::value_ptr(mat));
}

void ShaderProgram::setUniform(unsigned int id, const std::vector<glm::mat4> &mats)
{
    this->use();
    glUniformMatrix4fv(id, mats.size(), GL_FALSE, glm::value_ptr(*mats.data()));
}

void ShaderProgram::compileAndLinkShaders(const std::string &vertexShaderFile, const std::string &fragmentShaderFile)
{
    Shader vertexShader(vertexShaderFile, Shader::ShaderType::VertexShader);
    auto success = vertexShader.compile();
    (void) success;
    assert(success == true);
    addShader(vertexShader);

    Shader fragmentShader(fragmentShaderFile, Shader::ShaderType::FragmentShader);
    success = fragmentShader.compile();
    assert(success == true);
    addShader(fragmentShader);

    link();
}
