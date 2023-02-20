cmake_minimum_required(VERSION 3.14)

# generate the configuration header
set(CONFIG_HEADER ${PROJECT_BINARY_DIR}/src/config.h)
string(TIMESTAMP BUILD_DATE "%Y-%m-%d")
execute_process(
        COMMAND git log -1 --format=%h
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
        ${CONFIG_HEADER}
)

add_library(engine STATIC
        ${CONFIG_HEADER}
        include/engine/Engine.h
        include/engine/Scene.h
        include/engine/Window.h
        include/engine/DemoWindow.h
        include/engine/OverlayWindow.h
        include/engine/Renderer.h
        include/engine/Camera.h
        include/engine/Shader.h
        include/engine/Mesh.h
        include/engine/ShaderProgram.h
        include/engine/IndexBuffer.h
        include/engine/VertexBuffer.h
        Engine.cpp
        Scene.cpp
        Mesh.cpp
        Window.cpp
        DemoWindow.cpp
        OverlayWindow.cpp
        renderer/IndexBuffer.cpp
        renderer/VertexBuffer.cpp
        renderer/Shader.cpp
        renderer/Renderer.cpp
        renderer/ShaderProgram.cpp
        renderer/Camera.cpp
        )

target_include_directories(engine PUBLIC
        /usr/include/imgui
        ${PROJECT_SOURCE_DIR}/engine/include
        ${PROJECT_BINARY_DIR}/src
        )

target_link_libraries(engine -lglfw -lGL -lGLEW -lspdlog -lfmt -limgui -lstb -lassimp)
