#include <glm/glm.hpp>
#include <spdlog/spdlog.h>
#include <imgui/imgui.h>

#include <engine/Engine.h>
#include <engine/Scene.h>
#include <engine/Mesh.h>
#include <engine/Window.h>
#include <engine/Camera.h>
#include <engine/IndexBuffer.h>
#include <engine/VertexBuffer.h>
#include <engine/ShaderProgram.h>

namespace EngineApp {

    /**
     *  DemoSceneConfigWindow inherits from Engine::Window so as soon
     *  as the object is constructed, a GUI window will be rendered.
     *  The config parameters in this class are updated every frame.
     */
    class DemoSceneConfigWindow : public Engine::Window
    {
    public:
        DemoSceneConfigWindow()
                : Window("Demo Scene Config Window")
                  , m_cameraPos{1.0, 10.0, 3.0}
                  , m_cameraLookAt{0.0, 0.0, 0.0}
        {}

    protected:
        /**
         * By overriding renderContents() we can control what should be
         * rendered inside the GUI window. For now: raw ImGui controls...
         * This should maybe be encapsulated in the future ?
         */
        void renderContents() override
        {
            auto &io = ImGui::GetIO();
            ImGui::Text("Average real frame rate: %f FPS", io.Framerate);
            ImGui::SliderFloat("Framerate limit", &Engine::framerateLimit(), 1.0, 60.0);
            ImGui::Separator();
            ImGui::SliderFloat3("Camera Position", (float *) &m_cameraPos, -20.0, 20.0);
            ImGui::SliderFloat3("Camera Look-At", (float *) &m_cameraLookAt, -20.0, 20.0);
        }

    public:
        float m_cameraPos[3];
        float m_cameraLookAt[3];
    };


    /**
     * The DemoScene object inherits from Engine::Scene which means it is
     * a render-able scene. Engine::setScene() sets the currently active scene.
     */
    class DemoScene : public Engine::Scene
    {
    public:
        DemoScene()
                : Scene(std::make_shared<Engine::Renderer::Camera>(glm::vec3()))
                  , config(std::make_shared<DemoSceneConfigWindow>())
        {
            // compile a vertex and a fragment shader and link them into
            // a single shader program that we can use
            m_gridFloorShaderProgram.compileAndLinkShaders("../shader/demoVertexShader.glsl",
                                                           "../shader/demoFragmentShader.glsl");
            m_characterShader.compileAndLinkShaders("../shader/v_animatedCharacter.glsl",
                                                    "../shader/f_animatedCharacter.glsl");

            // after the shader program has been linked we query the ID
            // of the u_MVP uniform so we can upload the MVP matrix later...
            u_mvp = m_gridFloorShaderProgram.getUniformByName("u_MVP");
            u_mvpChar = m_characterShader.getUniformByName("u_MVP");

            // We want to render a tiled floor that spans a square area of 100 m2. To do
            // that we create a vertex buffer here in the constructor and fill it with
            // the vertices that make up our grid. We also use an index buffer to reduce
            // the amount of vertices in our vertex buffer.
            std::vector<glm::vec3> vertices;
            for (int x = -50; x < 50; x++)
            {
                for (int y = -50; y < 50; y++)
                {
                    // let's use the z-coordinate (which is always zero here) for color info
                    const auto color = ((x % 2) * (x % 2) == (y % 2) * (y % 2)) ? 1.0f : -1.0f;

                    // first triangle of the quad
                    vertices.push_back(glm::vec3(x, y, color));
                    vertices.push_back(glm::vec3(x + 1, y, color));
                    vertices.push_back(glm::vec3(x, y + 1, color));

                    // second triangle of the quad
                    vertices.push_back(glm::vec3(x + 1, y, color));
                    vertices.push_back(glm::vec3(x, y + 1, color));
                    vertices.push_back(glm::vec3(x + 1, y + 1, color));
                }
            }

            // upload our vertex data to the GPU
            m_vertexBuffer.addFloatLayoutAttribute(3);
            m_vertexBuffer.setData<glm::vec3>(vertices);

            if (!(m_characterMeshLoaded = m_characterMesh.read("../data/man_suit.fbx", true)))
            {
                spdlog::warn("cannot load character man_suit fbx file");
            }
        }

        void update() override
        {
            // at each frame update, we read the values from the configuration window
            // and update the camera's view matrix.

            getCamera()->setPosition(glm::vec3(config->m_cameraPos[0],
                                               config->m_cameraPos[1], config->m_cameraPos[2]));
            getCamera()->setLookAtPoint(glm::vec3(config->m_cameraLookAt[0],
                                                  config->m_cameraLookAt[1], config->m_cameraLookAt[2]));
        }

        void render(const glm::mat4 &projectionMatrix) override
        {
            // Compute the Model-View-Projection matrix and upload it to the shader
            // program on the GPU (model component has been left out for now)
            const auto MV = projectionMatrix * getViewMatrix();
            m_gridFloorShaderProgram.setUniform(u_mvp, MV);
            m_characterShader.setUniform(u_mvpChar, MV);

            // Draw contents of the vertex buffer as triangles
            m_gridFloorShaderProgram.use();
            m_vertexBuffer.drawTriangles();

            if (m_characterMeshLoaded)
            {
                m_characterShader.use();
                m_characterMesh.render();
            }
        }

    private:
        bool m_characterMeshLoaded;
        unsigned int u_mvp, u_mvpChar;
        Engine::Mesh m_characterMesh;
        Engine::Renderer::VertexBuffer m_vertexBuffer;
        Engine::Renderer::ShaderProgram m_gridFloorShaderProgram, m_characterShader;
        std::shared_ptr<DemoSceneConfigWindow> config;
    };

    /**
     * The onInit() function must be implemented by the user of the
     * framework to be used as an entry point to the application.
     */
    void onInit()
    {
        Engine::setScene(std::make_shared<DemoScene>());
    }
}
