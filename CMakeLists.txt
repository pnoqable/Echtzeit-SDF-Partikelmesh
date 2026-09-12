cmake_minimum_required(VERSION 3.20)
project(sdf-particle-mesh LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

include(FetchContent)

# Deterministischer Pfad fuer FetchContent-Abhaengigkeiten (wird u.a. vom
# raylib-PATCH_COMMAND benoetigt, um die geclonte Quelle zu finden).
set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_deps" CACHE STRING "" FORCE)

# raylib
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5
  GIT_SHALLOW    TRUE
  # Backport des raylib-master-Fixes (#4834/#1982): Nach Fenster-Resize liefert
  # GetScreenWidth()/GetScreenHeight() auf Windows/Linux mit HiDPI die physische
  # Pixelgroesse statt der logischen. Wird durch den FramebufferSizeCallback
  # korrigiert (siehe cmake/patches/RaylibHiDPIResizeFix.cmake).
  PATCH_COMMAND ${CMAKE_COMMAND} -DRAYLIB_SOURCE_DIR=${FETCHCONTENT_BASE_DIR}/raylib-src
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/RaylibHiDPIResizeFix.cmake
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

# GLM
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG        1.0.1
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glm)

# Dear ImGui
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.92.7
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

# rlImGui (raylib backend + high-level API, kompatibel mit raylib 5.5 / ImGui 1.92.7)
FetchContent_Declare(
  rlImGui
  GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
  GIT_TAG        Raylib_5_5
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(rlImGui)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/simulation/PrimitiveSDF.cpp
  src/simulation/ParticleSystem.cpp
  src/simulation/SpatialHash.cpp
  src/mesh/Triangulation.cpp
  src/render/SceneRenderer.cpp
  src/platform/SystemTheme.cpp
  src/debug/Metrics.cpp

  # ImGui core
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp

  # rlImGui (enthaelt Raylib-Backend + High-Level-API)
  ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
  ${imgui_SOURCE_DIR}
  ${rlimgui_SOURCE_DIR}
  ${rlimgui_SOURCE_DIR}/extras
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  raylib
  glm::glm
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  GRAPHICS_API_OPENGL_33
)

# macOS
if(APPLE)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    "-framework IOKit"
    "-framework Cocoa"
    "-framework OpenGL"
  )
endif()

# Linux
if(UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    m pthread dl rt X11
  )
endif()
