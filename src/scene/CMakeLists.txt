add_library(cadly_scene STATIC
  src/Camera.cpp
  src/DefaultView.cpp
  src/Scene.cpp
  src/Aabb.cpp
)
add_library(Cadly::Scene ALIAS cadly_scene)

target_include_directories(cadly_scene
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(cadly_scene PUBLIC cxx_std_17)

# Scene must NOT depend on Qt, OCCT, OpenGL, or Vulkan — only on glm and the
# logging shim (which itself only pulls in spdlog/fmt).
target_link_libraries(cadly_scene
  PUBLIC
    glm::glm
  PRIVATE
    Cadly::Platform
    Cadly::Warnings
)
