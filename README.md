# fbx2.1glb

FBX to glTF/GLB command-line converter — **no Autodesk FBX SDK required**.

Forked from [godotengine/FBX2glTF](https://github.com/godotengine/FBX2glTF) (which itself is a fork of [facebookincubator/FBX2glTF](https://github.com/facebookincubator/FBX2glTF)). The proprietary Autodesk FBX SDK has been completely replaced with [ufbx](https://github.com/ufbx/ufbx), a MIT-licensed single-file C99 FBX parser.

## Why this fork?

The original FBX2glTF depends on the Autodesk FBX SDK, which:

- Has **no ARM64 (Apple Silicon) support** — requires Rosetta 2 on macOS
- Is **proprietary** — cannot be freely redistributed
- Requires a **separate SDK download** and license agreement

This fork uses **ufbx** instead, enabling:

- **ARM64-native builds** on Apple Silicon and Linux ARM64
- **Fully open-source** — MIT/BSD, no proprietary dependencies
- **Single-step build** — no SDK downloads needed
- **C++17 standard library** — removed Boost dependency

## Features

- Mesh geometry (positions, normals, tangents, UVs, vertex colors)
- PBR metallic-roughness and traditional (Lambert/Phong) materials
- Embedded textures
- Skeletal skinning (bone weights, inverse bind matrices)
- Blend shapes / morph targets (with sparse accessor support)
- Lights (directional, point, spot) via KHR_lights_punctual
- Cameras (perspective, orthographic)
- Animation baking (24/30/60 fps)
- Draco mesh compression (optional)
- Binary (.glb) and JSON (.gltf) output

## Build

```bash
# Prerequisites: CMake 3.16+, C++17 compiler
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Binary at: build/fbx2glb
```

## Usage

```bash
# FBX to GLB (binary)
./build/fbx2glb -b -i model.fbx -o output

# FBX to glTF (JSON + separate files)
./build/fbx2glb -i model.fbx -o output

# With Draco compression
./build/fbx2glb -b --draco -i model.fbx -o output

# Verbose output
./build/fbx2glb -b -v -i model.fbx -o output
```

### Options

| Flag | Description |
|------|-------------|
| `-b, --binary` | Output binary .glb |
| `-v, --verbose` | Verbose logging |
| `-i, --input` | Input FBX file |
| `-o, --output` | Output path (without extension) |
| `--draco` | Enable Draco mesh compression |
| `--compute-normals` | `never\|broken\|missing\|always` |
| `--anim-framerate` | `bake24\|bake30\|bake60` |
| `--skinning-weights N` | Max bone influences per vertex (default: 8) |
| `--blend-shape-normals` | Include blend shape normals |
| `--no-khr-lights-punctual` | Disable light export |

## Dependencies

All vendored in `third_party/` — no external downloads needed:

| Library | License | Purpose |
|---------|---------|---------|
| [ufbx](https://github.com/ufbx/ufbx) | MIT / Public Domain | FBX parsing |
| [draco](https://github.com/google/draco) | Apache 2.0 | Mesh compression (fetched by CMake) |
| [fmt](https://github.com/fmtlib/fmt) | MIT | String formatting (fetched by CMake) |
| [mathfu](https://github.com/google/mathfu) | Apache 2.0 | Math types |
| [stb_image](https://github.com/nothings/stb) | MIT / Public Domain | Image I/O |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | JSON serialization |
| [CLI11](https://github.com/CLIUtils/CLI11) | BSD | Argument parsing |

## License

BSD 3-Clause License. See [LICENSE](LICENSE).

Unlike the original FBX2glTF, **no proprietary Autodesk license applies** — ufbx is fully open-source.
