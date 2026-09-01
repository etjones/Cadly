add_library(cadly_platform STATIC
  src/Log.cpp
  src/Paths.cpp
)
add_library(Cadly::Platform ALIAS cadly_platform)

target_include_directories(cadly_platform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(cadly_platform PUBLIC cxx_std_17)

# Paths.cpp consults this define so dev builds can locate shaders/assets
# under the source tree without needing an install step.
target_compile_definitions(cadly_platform
  PRIVATE
    CADLY_SOURCE_ROOT="${Cadly_SOURCE_DIR}"
)

target_link_libraries(cadly_platform
  PUBLIC
    spdlog::spdlog
    fmt::fmt
  PRIVATE
    Cadly::Warnings
)
