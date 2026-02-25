cmake_minimum_required(VERSION 3.14)
project(fbx2glb VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if ("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed. Use: cmake -B build")
endif()

include(FetchContent)
find_package(Threads REQUIRED)

# ==============================================================
# Dependencies
# ==============================================================

# --- ufbx (vendored single-file C99 FBX parser) ---
add_library(ufbx STATIC third_party/ufbx/ufbx.c)
target_include_directories(ufbx PUBLIC third_party/ufbx)
set_target_properties(ufbx PROPERTIES C_STANDARD 99)

# --- fmt ---
FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
  GIT_TAG 10.2.1
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(fmt)

# --- Draco (mesh compression, required for web delivery) ---
set(DRACO_TRANSCODER_SUPPORTED OFF CACHE BOOL "" FORCE)
set(DRACO_JS_GLUE OFF CACHE BOOL "" FORCE)
set(DRACO_TESTS OFF CACHE BOOL "" FORCE)
set(DRACO_ANIMATION_ENCODING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(draco
  GIT_REPOSITORY https://github.com/google/draco
  GIT_TAG 1.5.7
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(draco)

# --- MathFu (header-only math library) ---
# MathFu + vectorial (vendored in third_party/mathfu/)
set(MATHFU_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mathfu/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mathfu/dependencies/vectorial/include"
)

# --- FiFoMap (header-only ordered map for JSON) ---
FetchContent_Declare(fifo_map
  GIT_REPOSITORY https://github.com/nlohmann/fifo_map
  GIT_SHALLOW TRUE
)
FetchContent_Populate(fifo_map)

# --- CPPCodec (header-only base64 encoding) ---
FetchContent_Declare(cppcodec
  GIT_REPOSITORY https://github.com/tplgy/cppcodec
  GIT_SHALLOW TRUE
)
FetchContent_Populate(cppcodec)

# ==============================================================
# Source files
# ==============================================================

set(LIB_SOURCE_FILES
  src/FBX2glTF.h
  # fbx/ — ufbx-based FBX loader (replaces Autodesk FBX SDK)
  src/fbx/Fbx2Raw.cpp
  src/fbx/Fbx2Raw.hpp
  # gltf/ — glTF 2.0 output (unchanged from upstream)
  src/gltf/Raw2Gltf.cpp
  src/gltf/Raw2Gltf.hpp
  src/gltf/GltfModel.cpp
  src/gltf/GltfModel.hpp
  src/gltf/TextureBuilder.cpp
  src/gltf/TextureBuilder.hpp
  src/gltf/properties/AccessorData.cpp
  src/gltf/properties/AccessorData.hpp
  src/gltf/properties/AnimationData.cpp
  src/gltf/properties/AnimationData.hpp
  src/gltf/properties/BufferData.cpp
  src/gltf/properties/BufferData.hpp
  src/gltf/properties/BufferViewData.cpp
  src/gltf/properties/BufferViewData.hpp
  src/gltf/properties/CameraData.cpp
  src/gltf/properties/CameraData.hpp
  src/gltf/properties/ImageData.cpp
  src/gltf/properties/ImageData.hpp
  src/gltf/properties/LightData.cpp
  src/gltf/properties/LightData.hpp
  src/gltf/properties/MaterialData.cpp
  src/gltf/properties/MaterialData.hpp
  src/gltf/properties/MeshData.cpp
  src/gltf/properties/MeshData.hpp
  src/gltf/properties/NodeData.cpp
  src/gltf/properties/NodeData.hpp
  src/gltf/properties/PrimitiveData.cpp
  src/gltf/properties/PrimitiveData.hpp
  src/gltf/properties/SamplerData.hpp
  src/gltf/properties/SceneData.cpp
  src/gltf/properties/SceneData.hpp
  src/gltf/properties/SkinData.cpp
  src/gltf/properties/SkinData.hpp
  src/gltf/properties/TextureData.cpp
  src/gltf/properties/TextureData.hpp
  # math + raw model (FBX-SDK-free)
  src/mathfu.hpp
  src/raw/RawModel.cpp
  src/raw/RawModel.hpp
  # utils
  src/utils/File_Utils.cpp
  src/utils/File_Utils.hpp
  src/utils/Image_Utils.cpp
  src/utils/Image_Utils.hpp
  src/utils/String_Utils.hpp
)

# ==============================================================
# Targets
# ==============================================================

add_library(libfbx2glb STATIC ${LIB_SOURCE_FILES})
set_target_properties(libfbx2glb PROPERTIES OUTPUT_NAME "fbx2glb")

add_executable(fbx2glb src/FBX2glTF.cpp)
set_target_properties(fbx2glb PROPERTIES OUTPUT_NAME "fbx2glb")

target_link_libraries(libfbx2glb PUBLIC
  ufbx
  fmt::fmt
  draco_static
  Threads::Threads
  ${CMAKE_DL_LIBS}
)

# std::filesystem may need explicit linking on older compilers
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  target_link_libraries(libfbx2glb PUBLIC stdc++fs)
endif()

if (APPLE)
  find_library(CF_FRAMEWORK CoreFoundation)
  if (CF_FRAMEWORK)
    target_link_libraries(libfbx2glb PUBLIC ${CF_FRAMEWORK})
  endif()
endif()

target_include_directories(libfbx2glb PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(libfbx2glb SYSTEM PUBLIC
  "third_party/stb"
  "third_party/json"
  ${MATHFU_INCLUDE_DIRS}
  "${fifo_map_SOURCE_DIR}/src"
  "${cppcodec_SOURCE_DIR}"
  "${draco_SOURCE_DIR}/src"
  "${CMAKE_BINARY_DIR}"
)

target_include_directories(fbx2glb PUBLIC
  "third_party/CLI11"
)

target_link_libraries(fbx2glb libfbx2glb)

if (NOT MSVC)
  target_compile_options(libfbx2glb PRIVATE -Wunused)
  target_compile_options(fbx2glb PRIVATE -Wunused)
endif()

install(TARGETS fbx2glb
  RUNTIME DESTINATION bin
)
