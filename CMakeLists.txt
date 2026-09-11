cmake_minimum_required(VERSION 3.20)
project(sdf-particle-mesh LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

include(FetchContent)

# raylib
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.5
  GIT_SHALLOW    TRUE
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

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/simulation/PrimitiveSDF.cpp
  src/simulation/ParticleSystem.cpp
  src/render/SceneRenderer.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE src)

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
