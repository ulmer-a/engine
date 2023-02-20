#include <chrono>
#include <cassert>
#include <thread>
#include <list>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <spdlog/spdlog.h>
#include <imgui/imgui.h>
#include <imgui/backends/imgui_impl_glfw.h>
#include <imgui/backends/imgui_impl_opengl3.h>

#include "include/engine/Engine.h"
#include "include/engine/Scene.h"
#include "include/engine/Window.h"
#include "engine/Renderer.h"

namespace Engine {

    /**
     * The default framerate limit for the whole application.
     */
    static float s_framerateLimit = 30.0f;

    float &framerateLimit()
    {
        return s_framerateLimit;
    }

    /**
     * The GLFW window handle and defaults for it's size. The viewport
     * perspective projection matrix depends on the viewport's size and is
     * passed to subsequent render calls.
     */
    static GLFWwindow *s_mainWindow = nullptr;
    static glm::mat4 s_viewportProjectionMatrix;
    static std::shared_ptr<Scene> s_scene;
    static std::list<Window *> s_guiWindows;
    static unsigned int s_viewportWidth = 1024, s_viewportHeight = 768;


    static void updateViewportProjectionMatrix()
    {
        s_viewportProjectionMatrix = glm::perspective(
                glm::radians(45.0f), (float) s_viewportWidth / (float) s_viewportHeight, 1.0f, 1000.0f
        );
    }

    static void glfwResizeCallback(GLFWwindow *, int width, int height)
    {
        s_viewportWidth = width;
        s_viewportHeight = height;

        glViewport(0, 0, width, height);
        updateViewportProjectionMatrix();
    }

    static void setupGlfwWindow(const std::string &windowTitle)
    {
        if (!glfwInit())
        {
            spdlog::critical("glfwInit() failed");
            exit(EXIT_FAILURE);
        }

        // we are using multisampling-anti-aliasing (MSAA) with four samples
        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

        // here we could try to request a specific OpenGL version
        //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, OPENGL_VERSION_MAJOR);
        //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, OPENGL_VERSION_MINOR);

        s_mainWindow = glfwCreateWindow(s_viewportWidth, s_viewportHeight, windowTitle.c_str(), nullptr, nullptr);
        if (s_mainWindow == nullptr)
        {
            spdlog::critical("cannot create application window: glfwCreateWindow() failed");
            glfwTerminate();
            exit(EXIT_FAILURE);
        }

        glfwGetWindowSize(s_mainWindow, (int *) &s_viewportWidth, (int *) &s_viewportHeight);
        glfwSetWindowSizeCallback(s_mainWindow, glfwResizeCallback);
        updateViewportProjectionMatrix();

        glfwMakeContextCurrent(s_mainWindow);
    }

    static void terminateGlfw()
    {
        glfwDestroyWindow(s_mainWindow);
        glfwTerminate();
    }

    static void setupImGui()
    {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();

        ImGui_ImplGlfw_InitForOpenGL(s_mainWindow, true);
        ImGui_ImplOpenGL3_Init("#version 130");
    }

    static void terminateImGui()
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    static void glErrorCallback(
            GLenum source, GLenum type, GLuint id,
            GLenum severity, GLsizei length,
            const GLchar *message, const void *userParam
    )
    {
        (void) source;
        (void) length;
        (void) type;
        (void) severity;
        (void) userParam;

        spdlog::error("OpenGL error: {} (id={})", message, id);
    }

    static void setupOpenGl()
    {
        auto err = glewInit();
        if (err != GLEW_OK)
        {
            spdlog::critical("glewInit() failed (GLenum = {})", err);
            exit(EXIT_FAILURE);
        }

        int versionMajor, versionMinor;
        glGetIntegerv(GL_MAJOR_VERSION, &versionMajor);
        glGetIntegerv(GL_MINOR_VERSION, &versionMinor);
        spdlog::info("OpenGL version is {}.{}", versionMajor, versionMinor);

        glDebugMessageCallback(glErrorCallback, nullptr);
        glEnable(GL_MULTISAMPLE);
        glEnable(GL_DEPTH_TEST);

        Renderer::setClearColor(glm::vec3(0.0, 0.8, 1.0));
    }

    static void renderImGui()
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        for (auto &window: s_guiWindows)
        {
            assert(window != nullptr);
            window->render();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    static void update()
    {
        glfwPollEvents();

        if (s_scene != nullptr)
        {
            s_scene->update();
        }
    }

    static void renderFrame()
    {
        Renderer::clear();
        if (s_scene != nullptr)
        {
            s_scene->render(s_viewportProjectionMatrix);
        }
        renderImGui();
        glfwSwapBuffers(s_mainWindow);
    }

    extern "C" int main(int argc, char **argv)
    {
        (void) argc;
        (void) argv;

        spdlog::set_level(spdlog::level::debug);

        setupGlfwWindow("Ulmer 3D Game Engine");
        setupImGui();
        setupOpenGl();

        EngineApp::onInit();

        auto previousFrameCompleted = std::chrono::system_clock::now();
        while (!glfwWindowShouldClose(s_mainWindow))
        {
            update();
            renderFrame();

            // if a frame has taken less time than minFrameDuration to update and render, we wait until that
            // time has elapsed. this will limit the framerate to the amount specified by 's_framerateLimit'.
            const auto minFrameDuration = std::chrono::nanoseconds((long) (1000000000 / s_framerateLimit));
            std::this_thread::sleep_until(previousFrameCompleted + minFrameDuration);
            previousFrameCompleted += minFrameDuration;
        }

        terminateImGui();
        terminateGlfw();
        return EXIT_SUCCESS;
    }

    void setScene(std::shared_ptr<Scene> scene)
    {
        s_scene = scene;
    }

    namespace _internal {

        void registerWindow(Window *window)
        {
            s_guiWindows.push_back(window);
        }

        void unregisterWindow(Window *window)
        {
            s_guiWindows.remove(window);
        }
    }
} // Engine
