/**
 * fbx2.1glb — FBX to glTF/GLB converter using ufbx
 *
 * This file replaces the original Fbx2Raw.cpp that depended on
 * the proprietary Autodesk FBX SDK. It uses ufbx instead, a
 * MIT/Public Domain single-file C99 FBX parser.
 */

#include <ufbx.h>  // Must be included before Fbx2Raw.hpp so mathfu.hpp sees ufbx types

#include "Fbx2Raw.hpp"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "FBX2glTF.h"
#include "raw/RawModel.hpp"
#include "utils/File_Utils.hpp"
#include "utils/String_Utils.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// -------------------------------------------------------
// Texture finding
// -------------------------------------------------------

static std::string FindTextureFile(
    const std::string& textureFileName,
    const std::string& fbxFolder,
    const std::set<std::string>& textureExtensions) {
  namespace fs = std::filesystem;

  if (textureFileName.empty()) {
    return "";
  }

  // Try exact path first
  if (fs::exists(textureFileName)) {
    return fs::absolute(textureFileName).string();
  }

  // Extract just the filename
  fs::path texPath(textureFileName);
  std::string baseName = texPath.stem().string();
  std::string ext = texPath.extension().string();
  std::string fileName = texPath.filename().string();

  // Folders to search: fbx folder, fbm subfolder, current dir
  std::vector<std::string> folders;
  std::string fbmFolder = fbxFolder + "/" + fs::path(fbxFolder).stem().string() + ".fbm";
  if (fs::exists(fbmFolder) && fs::is_directory(fbmFolder)) {
    folders.push_back(fbmFolder);
  }
  folders.push_back(fbxFolder);
  folders.push_back(fs::current_path().string());

  for (const auto& folder : folders) {
    if (!fs::exists(folder) || !fs::is_directory(folder)) continue;

    // Try exact filename match
    fs::path candidate = fs::path(folder) / fileName;
    if (fs::exists(candidate)) {
      return fs::absolute(candidate).string();
    }

    // Try matching with different extensions
    for (const auto& entry : fs::directory_iterator(folder)) {
      if (!entry.is_regular_file()) continue;
      std::string entryBase = entry.path().stem().string();
      std::string entryExt = entry.path().extension().string();
      // Remove leading dot for extension comparison
      if (!entryExt.empty() && entryExt[0] == '.') entryExt = entryExt.substr(1);

      if (StringUtils::CompareNoCase(baseName, entryBase) == 0) {
        if (textureExtensions.empty() || textureExtensions.count(entryExt) > 0 ||
            textureExtensions.count(StringUtils::ToLower(entryExt)) > 0) {
          return fs::absolute(entry.path()).string();
        }
      }
    }
  }

  return "";
}

// -------------------------------------------------------
// Helpers
// -------------------------------------------------------

static bool TriangleTexturePolarity(const Vec2f& uv0, const Vec2f& uv1, const Vec2f& uv2) {
  const Vec2f d0 = uv1 - uv0;
  const Vec2f d1 = uv2 - uv0;
  return (d0[0] * d1[1] - d0[1] * d1[0] < 0.0f);
}

static RawMaterialType GetMaterialType(
    const RawModel& raw,
    const int textures[RAW_TEXTURE_USAGE_MAX],
    const bool vertexTransparency,
    const bool skinned) {
  int diffuseTexture = textures[RAW_TEXTURE_USAGE_DIFFUSE];
  if (diffuseTexture < 0) {
    diffuseTexture = textures[RAW_TEXTURE_USAGE_ALBEDO];
  }
  if (diffuseTexture >= 0) {
    return (raw.GetTexture(diffuseTexture).occlusion == RAW_TEXTURE_OCCLUSION_OPAQUE)
        ? (skinned ? RAW_MATERIAL_TYPE_SKINNED_OPAQUE : RAW_MATERIAL_TYPE_OPAQUE)
        : (skinned ? RAW_MATERIAL_TYPE_SKINNED_TRANSPARENT : RAW_MATERIAL_TYPE_TRANSPARENT);
  }
  if (vertexTransparency) {
    return skinned ? RAW_MATERIAL_TYPE_SKINNED_TRANSPARENT : RAW_MATERIAL_TYPE_TRANSPARENT;
  }
  return skinned ? RAW_MATERIAL_TYPE_SKINNED_OPAQUE : RAW_MATERIAL_TYPE_OPAQUE;
}

static std::string ufbxStr(const ufbx_string& s) {
  return std::string(s.data, s.length);
}

// Apply a ufbx_matrix to a ufbx_vec3 (affine transform)
static ufbx_vec3 transformPoint(const ufbx_matrix& m, const ufbx_vec3& v) {
  ufbx_vec3 r;
  r.x = m.cols[0].x * v.x + m.cols[1].x * v.y + m.cols[2].x * v.z + m.cols[3].x;
  r.y = m.cols[0].y * v.x + m.cols[1].y * v.y + m.cols[2].y * v.z + m.cols[3].y;
  r.z = m.cols[0].z * v.x + m.cols[1].z * v.y + m.cols[2].z * v.z + m.cols[3].z;
  return r;
}

// Apply rotation/scale part of a matrix to a direction (no translation)
static ufbx_vec3 transformDir(const ufbx_matrix& m, const ufbx_vec3& v) {
  ufbx_vec3 r;
  r.x = m.cols[0].x * v.x + m.cols[1].x * v.y + m.cols[2].x * v.z;
  r.y = m.cols[0].y * v.x + m.cols[1].y * v.y + m.cols[2].y * v.z;
  r.z = m.cols[0].z * v.x + m.cols[1].z * v.y + m.cols[2].z * v.z;
  return r;
}

static float vecLen(const ufbx_vec3& v) {
  return sqrtf((float)(v.x * v.x + v.y * v.y + v.z * v.z));
}

static ufbx_vec3 normalizeVec(const ufbx_vec3& v) {
  float len = vecLen(v);
  if (len < 1e-12f) return {0, 0, 0};
  return {v.x / len, v.y / len, v.z / len};
}

// Find skeleton root: common ancestor of all bone nodes in a skin deformer
static ufbx_node* findSkeletonRoot(ufbx_skin_deformer* skin) {
  if (skin->clusters.count == 0) return nullptr;

  // Start with the first bone's parent chain
  ufbx_node* root = skin->clusters.data[0]->bone_node;
  if (!root) return nullptr;

  // Walk up to find the topmost bone
  while (root->parent && root->parent->bone) {
    root = root->parent;
  }
  // Check if the parent is a better root (contains all bones)
  if (root->parent) {
    root = root->parent;
  }
  return root;
}

// -------------------------------------------------------
// ReadNodeHierarchy
// -------------------------------------------------------
static void ReadNodeHierarchy(
    RawModel& raw,
    ufbx_scene* scene,
    ufbx_node* node,
    const long parentId,
    const std::string& path) {
  const long nodeId = (long)node->element.element_id;
  const char* nodeName = node->name.length > 0 ? node->name.data : "unnamed";

  const int nodeIndex = raw.AddNode(nodeId, nodeName, parentId, -1);
  RawNode& rawNode = raw.GetNode(nodeIndex);

  std::string newPath = path + "/" + nodeName;
  if (verboseOutput) {
    fmt::printf("node %d: %s\n", nodeIndex, newPath.c_str());
  }

  // Set local transform
  // ufbx with target_unit_meters=1.0 already converts to meters
  const ufbx_transform& lt = node->local_transform;
  rawNode.translation = toVec3f(lt.translation);
  rawNode.rotation = toQuatf(lt.rotation);
  rawNode.scale = toVec3f(lt.scale);

  if (parentId) {
    int parentIdx = raw.GetNodeById(parentId);
    if (parentIdx >= 0) {
      RawNode& parentNode = raw.GetNode(parentIdx);
      if (std::find(parentNode.childIds.begin(), parentNode.childIds.end(), nodeId) ==
          parentNode.childIds.end()) {
        parentNode.childIds.push_back(nodeId);
      }
    }
  } else {
    raw.SetRootNode(nodeId);
  }

  for (size_t i = 0; i < node->children.count; i++) {
    ReadNodeHierarchy(raw, scene, node->children.data[i], nodeId, newPath);
  }
}

// -------------------------------------------------------
// ReadMesh
// -------------------------------------------------------
static void ReadMesh(
    RawModel& raw,
    ufbx_scene* scene,
    ufbx_node* node,
    const std::string& fbxFolder,
    const std::set<std::string>& textureExtensions) {
  ufbx_mesh* mesh = node->mesh;
  if (!mesh) return;

  const long surfaceId = (long)mesh->element.element_id;
  const long nodeId = (long)node->element.element_id;

  // Associate node with surface
  int nodeIdx = raw.GetNodeById(nodeId);
  if (nodeIdx >= 0) {
    RawNode& rawNode = raw.GetNode(nodeIdx);
    rawNode.surfaceId = surfaceId;
  }

  // Skip if already loaded
  if (raw.GetSurfaceById(surfaceId) >= 0) return;

  const char* meshName = node->name.length > 0 ? node->name.data : mesh->name.data;
  const int rawSurfaceIndex = raw.AddSurface(meshName, surfaceId);

  // Determine which vertex attributes are present
  raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_POSITION);
  if (mesh->vertex_normal.exists) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_NORMAL);
  if (mesh->vertex_tangent.exists) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_TANGENT);
  if (mesh->vertex_bitangent.exists) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_BINORMAL);
  if (mesh->vertex_color.exists) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_COLOR);
  if (mesh->vertex_uv.exists) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_UV0);
  if (mesh->uv_sets.count > 1) raw.AddVertexAttribute(RAW_VERTEX_ATTRIBUTE_UV1);

  RawSurface& rawSurface = raw.GetSurface(rawSurfaceIndex);

  // Geometric transform: applied to vertex positions since glTF doesn't support it
  const ufbx_matrix& geoTransform = node->geometry_to_node;
  const bool hasGeoTransform = memcmp(&geoTransform, &ufbx_identity_matrix, sizeof(ufbx_matrix)) != 0;
  ufbx_matrix normalXform = geoTransform;

  // Skinning setup
  ufbx_skin_deformer* skin = mesh->skin_deformers.count > 0 ? mesh->skin_deformers.data[0] : nullptr;
  const bool isSkinned = (skin != nullptr && skin->clusters.count > 0);

  if (isSkinned) {
    ufbx_node* skelRoot = findSkeletonRoot(skin);
    rawSurface.skeletonRootId = skelRoot ? (long)skelRoot->element.element_id : nodeId;
  } else {
    rawSurface.skeletonRootId = nodeId;
  }

  // Build joint data for skinned meshes
  std::unordered_map<uint32_t, int> clusterBoneToJointIndex;
  if (isSkinned) {
    for (size_t ci = 0; ci < skin->clusters.count; ci++) {
      ufbx_skin_cluster* cluster = skin->clusters.data[ci];
      if (!cluster->bone_node) continue;

      long jointId = (long)cluster->bone_node->element.element_id;
      int jointNodeIdx = raw.GetNodeById(jointId);
      if (jointNodeIdx >= 0) {
        raw.GetNode(jointNodeIdx).isJoint = true;
      }

      clusterBoneToJointIndex[(uint32_t)ci] = (int)rawSurface.jointIds.size();
      rawSurface.jointIds.emplace_back(jointId);
      rawSurface.inverseBindMatrices.push_back(toMat4f(cluster->geometry_to_bone));
      rawSurface.jointGeometryMins.emplace_back(FLT_MAX, FLT_MAX, FLT_MAX);
      rawSurface.jointGeometryMaxs.emplace_back(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
  }

  if (verboseOutput) {
    fmt::printf(
        "mesh %d: %s (skinned: %s)\n",
        rawSurfaceIndex,
        meshName,
        isSkinned ? "YES" : "NO");
  }

  // Blend shapes
  rawSurface.blendChannels.clear();
  std::vector<ufbx_blend_shape*> targetShapes;
  for (size_t di = 0; di < mesh->blend_deformers.count; di++) {
    ufbx_blend_deformer* blend = mesh->blend_deformers.data[di];
    for (size_t ci = 0; ci < blend->channels.count; ci++) {
      ufbx_blend_channel* channel = blend->channels.data[ci];
      if (channel->target_shape) {
        targetShapes.push_back(channel->target_shape);
        rawSurface.blendChannels.push_back(RawBlendChannel{
            (float)channel->weight,
            channel->target_shape->normal_offsets.count > 0,
            false, // ufbx doesn't provide per-shape tangent offsets directly
            ufbxStr(channel->name)});
      }
    }
  }

  // Build per-vertex skinning lookup from ufbx_skin_deformer
  // skin->vertices[vertexIndex] → { weight_begin, num_weights }
  // skin->weights[weight_begin..] → { cluster_index, weight }

  // Build blend shape offset maps for quick lookup per control-point
  // blend_shape->offset_vertices[i] → vertex index, position_offsets[i] → offset
  std::vector<std::unordered_map<uint32_t, size_t>> blendOffsetMaps;
  for (auto* shape : targetShapes) {
    std::unordered_map<uint32_t, size_t> offsetMap;
    for (size_t oi = 0; oi < shape->offset_vertices.count; oi++) {
      offsetMap[shape->offset_vertices.data[oi]] = oi;
    }
    blendOffsetMaps.push_back(std::move(offsetMap));
  }

  // Allocate triangulation buffer
  std::vector<uint32_t> triIndices(mesh->max_face_triangles * 3);

  // Iterate over faces, triangulate, and emit vertices/triangles
  for (size_t fi = 0; fi < mesh->num_faces; fi++) {
    ufbx_face face = mesh->faces.data[fi];
    if (face.num_indices < 3) continue;

    // Get material for this face
    uint32_t matIdx = 0;
    if (mesh->face_material.count > fi) {
      matIdx = mesh->face_material.data[fi];
    }

    ufbx_material* ufbxMat = nullptr;
    if (matIdx < mesh->materials.count) {
      ufbxMat = mesh->materials.data[matIdx];
    }

    // Build material properties
    int textures[RAW_TEXTURE_USAGE_MAX];
    std::fill_n(textures, (int)RAW_TEXTURE_USAGE_MAX, -1);

    std::shared_ptr<RawMatProps> rawMatProps;
    std::string materialName = "DefaultMaterial";
    long materialId = -1;
    std::vector<std::string> userProperties;

    if (ufbxMat == nullptr) {
      rawMatProps.reset(new RawTraditionalMatProps(
          RAW_SHADING_MODEL_LAMBERT,
          Vec3f(0, 0, 0),
          Vec4f(.5, .5, .5, 1),
          Vec3f(0, 0, 0),
          Vec3f(0, 0, 0),
          0.5));
    } else {
      materialName = ufbxStr(ufbxMat->name);
      materialId = (long)ufbxMat->element.element_id;

      auto maybeAddTexture = [&](const ufbx_material_map& map, RawTextureUsage usage) {
        if (map.texture && map.texture->type == UFBX_TEXTURE_FILE) {
          std::string texName = ufbxStr(map.texture->name);
          std::string texFileName = ufbxStr(map.texture->filename);
          std::string texFileLoc = FindTextureFile(
              ufbxStr(map.texture->absolute_filename), fbxFolder, textureExtensions);
          if (texFileLoc.empty()) {
            texFileLoc = FindTextureFile(
                ufbxStr(map.texture->relative_filename), fbxFolder, textureExtensions);
          }
          if (texFileLoc.empty()) {
            texFileLoc = FindTextureFile(texFileName, fbxFolder, textureExtensions);
          }
          textures[usage] = raw.AddTexture(texName, texFileName, texFileLoc, usage);
        }
      };

      // Detect PBR vs traditional based on shader type
      if (ufbxMat->shader_type == UFBX_SHADER_UNKNOWN &&
          (StringUtils::CompareNoCase(ufbxStr(ufbxMat->shading_model_name), "lambert") == 0 ||
           StringUtils::CompareNoCase(ufbxStr(ufbxMat->shading_model_name), "phong") == 0 ||
           StringUtils::CompareNoCase(ufbxStr(ufbxMat->shading_model_name), "blinn") == 0 ||
           StringUtils::CompareNoCase(ufbxStr(ufbxMat->shading_model_name), "constant") == 0)) {
        // Traditional FBX material
        RawShadingModel shadingModel;
        std::string modelName = ufbxStr(ufbxMat->shading_model_name);
        if (StringUtils::CompareNoCase(modelName, "lambert") == 0) {
          shadingModel = RAW_SHADING_MODEL_LAMBERT;
        } else if (StringUtils::CompareNoCase(modelName, "blinn") == 0) {
          shadingModel = RAW_SHADING_MODEL_BLINN;
        } else if (StringUtils::CompareNoCase(modelName, "phong") == 0) {
          shadingModel = RAW_SHADING_MODEL_PHONG;
        } else if (StringUtils::CompareNoCase(modelName, "constant") == 0) {
          shadingModel = RAW_SHADING_MODEL_CONSTANT;
        } else {
          shadingModel = RAW_SHADING_MODEL_UNKNOWN;
        }

        maybeAddTexture(ufbxMat->fbx.diffuse_color, RAW_TEXTURE_USAGE_DIFFUSE);
        maybeAddTexture(ufbxMat->fbx.normal_map, RAW_TEXTURE_USAGE_NORMAL);
        maybeAddTexture(ufbxMat->fbx.emission_color, RAW_TEXTURE_USAGE_EMISSIVE);
        maybeAddTexture(ufbxMat->fbx.specular_color, RAW_TEXTURE_USAGE_SPECULAR);
        maybeAddTexture(ufbxMat->fbx.ambient_color, RAW_TEXTURE_USAGE_AMBIENT);

        Vec3f ambient(
            (float)ufbxMat->fbx.ambient_color.value_vec4.x,
            (float)ufbxMat->fbx.ambient_color.value_vec4.y,
            (float)ufbxMat->fbx.ambient_color.value_vec4.z);
        Vec4f diffuse(
            (float)ufbxMat->fbx.diffuse_color.value_vec4.x,
            (float)ufbxMat->fbx.diffuse_color.value_vec4.y,
            (float)ufbxMat->fbx.diffuse_color.value_vec4.z,
            1.0f - (float)ufbxMat->fbx.transparency_factor.value_real);
        Vec3f emissive(
            (float)ufbxMat->fbx.emission_color.value_vec4.x,
            (float)ufbxMat->fbx.emission_color.value_vec4.y,
            (float)ufbxMat->fbx.emission_color.value_vec4.z);
        Vec3f specular(
            (float)ufbxMat->fbx.specular_color.value_vec4.x,
            (float)ufbxMat->fbx.specular_color.value_vec4.y,
            (float)ufbxMat->fbx.specular_color.value_vec4.z);
        float shininess = (float)ufbxMat->fbx.specular_exponent.value_real;

        rawMatProps.reset(new RawTraditionalMatProps(
            shadingModel,
            std::move(ambient),
            std::move(diffuse),
            std::move(emissive),
            std::move(specular),
            shininess));
      } else {
        // PBR material (any other shader type, or autodetect)
        maybeAddTexture(ufbxMat->pbr.base_color, RAW_TEXTURE_USAGE_ALBEDO);
        maybeAddTexture(ufbxMat->pbr.normal_map, RAW_TEXTURE_USAGE_NORMAL);
        maybeAddTexture(ufbxMat->pbr.emission_color, RAW_TEXTURE_USAGE_EMISSIVE);
        maybeAddTexture(ufbxMat->pbr.roughness, RAW_TEXTURE_USAGE_ROUGHNESS);
        maybeAddTexture(ufbxMat->pbr.metalness, RAW_TEXTURE_USAGE_METALLIC);
        maybeAddTexture(ufbxMat->pbr.ambient_occlusion, RAW_TEXTURE_USAGE_OCCLUSION);

        Vec4f baseColor(
            (float)ufbxMat->pbr.base_color.value_vec4.x,
            (float)ufbxMat->pbr.base_color.value_vec4.y,
            (float)ufbxMat->pbr.base_color.value_vec4.z,
            (float)ufbxMat->pbr.base_color.value_vec4.w);
        Vec3f emissiveColor(
            (float)ufbxMat->pbr.emission_color.value_vec4.x,
            (float)ufbxMat->pbr.emission_color.value_vec4.y,
            (float)ufbxMat->pbr.emission_color.value_vec4.z);
        float emissiveIntensity = (float)ufbxMat->pbr.emission_factor.value_real;
        float metallic = (float)ufbxMat->pbr.metalness.value_real;
        float roughness = (float)ufbxMat->pbr.roughness.value_real;

        rawMatProps.reset(new RawMetRoughMatProps(
            RAW_SHADING_MODEL_PBR_MET_ROUGH,
            std::move(baseColor),
            std::move(emissiveColor),
            emissiveIntensity,
            metallic,
            roughness,
            false)); // invertRoughnessMap
      }
    }

    // Triangulate the face
    uint32_t numTris = ufbx_triangulate_face(
        triIndices.data(), triIndices.size(), mesh, face);

    // Process each triangle
    for (uint32_t ti = 0; ti < numTris; ti++) {
      RawVertex rawVertices[3];
      bool vertexTransparency = false;

      for (int vi = 0; vi < 3; vi++) {
        uint32_t idx = triIndices[ti * 3 + vi]; // index into mesh arrays
        uint32_t vertexIdx = mesh->vertex_indices.data[idx]; // control point index

        RawVertex& vertex = rawVertices[vi];

        // Position
        ufbx_vec3 pos = mesh->vertex_position.values.data[mesh->vertex_position.indices.data[idx]];
        if (hasGeoTransform) {
          pos = transformPoint(geoTransform, pos);
        }
        vertex.position[0] = (float)pos.x;
        vertex.position[1] = (float)pos.y;
        vertex.position[2] = (float)pos.z;

        // Normal
        if (mesh->vertex_normal.exists) {
          ufbx_vec3 norm = mesh->vertex_normal.values.data[mesh->vertex_normal.indices.data[idx]];
          if (hasGeoTransform) {
            norm = normalizeVec(transformDir(normalXform, norm));
          }
          vertex.normal[0] = (float)norm.x;
          vertex.normal[1] = (float)norm.y;
          vertex.normal[2] = (float)norm.z;
        }

        // Tangent
        if (mesh->vertex_tangent.exists) {
          ufbx_vec3 tan = mesh->vertex_tangent.values.data[mesh->vertex_tangent.indices.data[idx]];
          if (hasGeoTransform) {
            tan = normalizeVec(transformDir(normalXform, tan));
          }
          vertex.tangent[0] = (float)tan.x;
          vertex.tangent[1] = (float)tan.y;
          vertex.tangent[2] = (float)tan.z;
          vertex.tangent[3] = 1.0f; // handedness
        }

        // Bitangent / Binormal
        if (mesh->vertex_bitangent.exists) {
          ufbx_vec3 bn = mesh->vertex_bitangent.values.data[mesh->vertex_bitangent.indices.data[idx]];
          if (hasGeoTransform) {
            bn = normalizeVec(transformDir(normalXform, bn));
          }
          vertex.binormal[0] = (float)bn.x;
          vertex.binormal[1] = (float)bn.y;
          vertex.binormal[2] = (float)bn.z;
        }

        // Vertex color
        if (mesh->vertex_color.exists) {
          ufbx_vec4 col = mesh->vertex_color.values.data[mesh->vertex_color.indices.data[idx]];
          vertex.color[0] = (float)col.x;
          vertex.color[1] = (float)col.y;
          vertex.color[2] = (float)col.z;
          vertex.color[3] = (float)col.w;
          vertexTransparency |= (fabs(col.w - 1.0) > 1e-3);
        }

        // UV0
        if (mesh->vertex_uv.exists) {
          ufbx_vec2 uv = mesh->vertex_uv.values.data[mesh->vertex_uv.indices.data[idx]];
          vertex.uv0[0] = (float)uv.x;
          vertex.uv0[1] = (float)uv.y;
        }

        // UV1
        if (mesh->uv_sets.count > 1) {
          const ufbx_uv_set& uvSet1 = mesh->uv_sets.data[1];
          if (uvSet1.vertex_uv.exists) {
            ufbx_vec2 uv = uvSet1.vertex_uv.values.data[uvSet1.vertex_uv.indices.data[idx]];
            vertex.uv1[0] = (float)uv.x;
            vertex.uv1[1] = (float)uv.y;
          }
        }

        // Skinning
        if (isSkinned && vertexIdx < skin->vertices.count) {
          const ufbx_skin_vertex& sv = skin->vertices.data[vertexIdx];
          for (uint32_t wi = 0; wi < sv.num_weights; wi++) {
            const ufbx_skin_weight& sw = skin->weights.data[sv.weight_begin + wi];
            auto it = clusterBoneToJointIndex.find(sw.cluster_index);
            if (it != clusterBoneToJointIndex.end() && sw.weight > 0.0f) {
              vertex.skinningInfo.push_back(
                  RawVertexSkinningInfo{it->second, (float)sw.weight});
            }
          }
        }

        vertex.polarityUv0 = false;

        rawSurface.bounds.AddPoint(vertex.position);

        // Blend shapes
        if (!targetShapes.empty()) {
          vertex.blendSurfaceIx = rawSurfaceIndex;
          for (size_t si = 0; si < targetShapes.size(); si++) {
            RawBlendVertex blendVertex;
            auto it = blendOffsetMaps[si].find(vertexIdx);
            if (it != blendOffsetMaps[si].end()) {
              size_t oi = it->second;
              const ufbx_blend_shape* shape = targetShapes[si];
              ufbx_vec3 posOff = shape->position_offsets.data[oi];
              if (hasGeoTransform) {
                posOff = transformDir(geoTransform, posOff);
              }
              blendVertex.position = toVec3f(posOff);
              if (oi < shape->normal_offsets.count) {
                ufbx_vec3 normOff = shape->normal_offsets.data[oi];
                if (hasGeoTransform) {
                  normOff = transformDir(normalXform, normOff);
                }
                blendVertex.normal = toVec3f(normOff);
              }
            }
            vertex.blends.push_back(blendVertex);
          }
        } else {
          vertex.blendSurfaceIx = -1;
        }

        // Skinning min/max for joint bounds
        if (isSkinned && !vertex.skinningInfo.empty()) {
          ufbx_vec3 ufbxPos = {vertex.position[0], vertex.position[1], vertex.position[2]};
          for (const auto& si : vertex.skinningInfo) {
            if (si.jointWeight > 0.0f && si.jointIndex < (int)rawSurface.jointIds.size()) {
              // Transform position to bone local space using inverse bind matrix
              ufbx_skin_cluster* cluster = nullptr;
              for (size_t ci = 0; ci < skin->clusters.count; ci++) {
                auto it2 = clusterBoneToJointIndex.find((uint32_t)ci);
                if (it2 != clusterBoneToJointIndex.end() && it2->second == si.jointIndex) {
                  cluster = skin->clusters.data[ci];
                  break;
                }
              }
              if (cluster) {
                ufbx_vec3 localPos = transformPoint(cluster->geometry_to_bone, ufbxPos);
                Vec3f& mins = rawSurface.jointGeometryMins[si.jointIndex];
                mins[0] = std::min(mins[0], (float)localPos.x);
                mins[1] = std::min(mins[1], (float)localPos.y);
                mins[2] = std::min(mins[2], (float)localPos.z);
                Vec3f& maxs = rawSurface.jointGeometryMaxs[si.jointIndex];
                maxs[0] = std::max(maxs[0], (float)localPos.x);
                maxs[1] = std::max(maxs[1], (float)localPos.y);
                maxs[2] = std::max(maxs[2], (float)localPos.z);
              }
            }
          }
        }
      } // end vertex loop

      // Set UV polarity for normal mapping
      if (textures[RAW_TEXTURE_USAGE_NORMAL] != -1) {
        const bool polarity =
            TriangleTexturePolarity(rawVertices[0].uv0, rawVertices[1].uv0, rawVertices[2].uv0);
        rawVertices[0].polarityUv0 = polarity;
        rawVertices[1].polarityUv0 = polarity;
        rawVertices[2].polarityUv0 = polarity;
      }

      int rawVertexIndices[3];
      for (int vi = 0; vi < 3; vi++) {
        rawVertexIndices[vi] = raw.AddVertex(rawVertices[vi]);
      }

      const RawMaterialType materialType =
          GetMaterialType(raw, textures, vertexTransparency, isSkinned);
      const int rawMaterialIndex = raw.AddMaterial(
          materialId,
          materialName.c_str(),
          materialType,
          textures,
          rawMatProps,
          userProperties,
          false);

      raw.AddTriangle(
          rawVertexIndices[0],
          rawVertexIndices[1],
          rawVertexIndices[2],
          rawMaterialIndex,
          rawSurfaceIndex);
    } // end triangle loop
  } // end face loop
}

// -------------------------------------------------------
// ReadLight
// -------------------------------------------------------
static void ReadLight(RawModel& raw, ufbx_node* node) {
  ufbx_light* light = node->light;
  if (!light) return;

  int lightIx;
  float intensity = (float)light->intensity;
  Vec3f color((float)light->color.x, (float)light->color.y, (float)light->color.z);

  switch (light->type) {
    case UFBX_LIGHT_DIRECTIONAL:
      lightIx = raw.AddLight(light->name.data, RAW_LIGHT_TYPE_DIRECTIONAL, color, intensity, 0, 0);
      break;
    case UFBX_LIGHT_POINT:
      lightIx = raw.AddLight(light->name.data, RAW_LIGHT_TYPE_POINT, color, intensity, 0, 0);
      break;
    case UFBX_LIGHT_SPOT:
      lightIx = raw.AddLight(
          light->name.data,
          RAW_LIGHT_TYPE_SPOT,
          color,
          intensity,
          (float)light->inner_angle * (float)M_PI / 180.0f,
          (float)light->outer_angle * (float)M_PI / 180.0f);
      break;
    default:
      fmt::printf("Warning: Ignoring unsupported light type.\n");
      return;
  }

  int nodeIdx = raw.GetNodeById((long)node->element.element_id);
  if (nodeIdx >= 0) {
    RawNode& rawNode = raw.GetNode(nodeIdx);
    rawNode.lightIx = lightIx;
  }
}

// -------------------------------------------------------
// ReadCamera
// -------------------------------------------------------
static void ReadCamera(RawModel& raw, ufbx_node* node) {
  ufbx_camera* camera = node->camera;
  if (!camera) return;

  long nodeId = (long)node->element.element_id;

  if (camera->projection_mode == UFBX_PROJECTION_MODE_PERSPECTIVE) {
    float aspectRatio = (float)camera->aspect_ratio;
    float fovX = (float)camera->field_of_view_deg.x;
    float fovY = (float)camera->field_of_view_deg.y;
    float nearZ = (float)camera->near_plane;
    float farZ = (float)camera->far_plane;

    raw.AddCameraPerspective("", nodeId, aspectRatio, fovX, fovY, nearZ, farZ);
  } else {
    float magX = (float)camera->orthographic_size.x;
    float magY = (float)camera->orthographic_size.y;
    float nearZ = (float)camera->near_plane;
    float farZ = (float)camera->far_plane;

    raw.AddCameraOrthographic("", nodeId, magX, magY, nearZ, farZ);
  }

  // Cameras in FBX face +X, glTF needs -Z. Apply -90 Y rotation.
  auto nodeIdx = raw.GetNodeById(nodeId);
  if (nodeIdx >= 0) {
    auto& rawNode = raw.GetNode(nodeIdx);
    auto r = Quatf::FromAngleAxis(-90.0f * ((float)M_PI / 180.0f), {0.0, 1.0, 0.0});
    rawNode.rotation = rawNode.rotation * r;
  }
}

// -------------------------------------------------------
// ReadNodeAttributes (recursive)
// -------------------------------------------------------
static void ReadNodeAttributes(
    RawModel& raw,
    ufbx_scene* scene,
    ufbx_node* node,
    const std::string& fbxFolder,
    const std::set<std::string>& textureExtensions) {
  if (!node->visible) return;

  if (node->mesh) {
    ReadMesh(raw, scene, node, fbxFolder, textureExtensions);
  }
  if (node->camera) {
    ReadCamera(raw, node);
  }
  if (node->light) {
    ReadLight(raw, node);
  }

  for (size_t i = 0; i < node->children.count; i++) {
    ReadNodeAttributes(raw, scene, node->children.data[i], fbxFolder, textureExtensions);
  }
}

// -------------------------------------------------------
// ReadAnimations
// -------------------------------------------------------
static void ReadAnimations(RawModel& raw, ufbx_scene* scene, const GltfOptions& options) {
  double fps = 30.0;
  switch (options.animationFramerate) {
    case AnimationFramerateOptions::BAKE24: fps = 24.0; break;
    case AnimationFramerateOptions::BAKE30: fps = 30.0; break;
    case AnimationFramerateOptions::BAKE60: fps = 60.0; break;
  }
  const double epsilon = 1e-5;
  const double frameDuration = 1.0 / fps;

  for (size_t animIx = 0; animIx < scene->anim_stacks.count; animIx++) {
    ufbx_anim_stack* animStack = scene->anim_stacks.data[animIx];
    std::string animName = ufbxStr(animStack->name);

    double timeBegin = animStack->time_begin;
    double timeEnd = animStack->time_end;

    if (timeEnd <= timeBegin) continue;

    RawAnimation animation;
    animation.name = animName;

    // Calculate frame times
    int numFrames = (int)ceil((timeEnd - timeBegin) * fps) + 1;
    for (int fi = 0; fi < numFrames; fi++) {
      double t = (double)fi * frameDuration;
      if (t > (timeEnd - timeBegin)) t = timeEnd - timeBegin;
      animation.times.push_back((float)t);
    }

    fmt::printf("Animation %s: [%.3f - %.3f] (%d frames)\n",
        animName, timeBegin, timeEnd, numFrames);

    if (verboseOutput) {
      fmt::printf("animation %zu: %s (0%%)", animIx, animName.c_str());
    }

    size_t totalSizeInBytes = 0;

    // Evaluate each node's transform at each frame
    for (size_t ni = 0; ni < scene->nodes.count; ni++) {
      ufbx_node* node = scene->nodes.data[ni];
      int rawNodeIdx = raw.GetNodeById((long)node->element.element_id);
      if (rawNodeIdx < 0) continue;

      // Get base (rest) transform
      const ufbx_transform& baseXform = node->local_transform;
      bool hasTranslation = false;
      bool hasRotation = false;
      bool hasScale = false;

      RawChannel channel;
      channel.nodeIndex = rawNodeIdx;

      for (int fi = 0; fi < numFrames; fi++) {
        double evalTime = timeBegin + (double)fi * frameDuration;
        if (evalTime > timeEnd) evalTime = timeEnd;

        // Evaluate animated transform
        ufbx_scene* evalScene = ufbx_evaluate_scene(scene, animStack->anim, evalTime, nullptr, nullptr);
        if (!evalScene) continue;

        ufbx_node* evalNode = evalScene->nodes.data[ni];
        const ufbx_transform& lt = evalNode->local_transform;

        hasTranslation |=
            (fabs(lt.translation.x - baseXform.translation.x) > epsilon ||
             fabs(lt.translation.y - baseXform.translation.y) > epsilon ||
             fabs(lt.translation.z - baseXform.translation.z) > epsilon);
        hasRotation |=
            (fabs(lt.rotation.x - baseXform.rotation.x) > epsilon ||
             fabs(lt.rotation.y - baseXform.rotation.y) > epsilon ||
             fabs(lt.rotation.z - baseXform.rotation.z) > epsilon ||
             fabs(lt.rotation.w - baseXform.rotation.w) > epsilon);
        hasScale |=
            (fabs(lt.scale.x - baseXform.scale.x) > epsilon ||
             fabs(lt.scale.y - baseXform.scale.y) > epsilon ||
             fabs(lt.scale.z - baseXform.scale.z) > epsilon);

        channel.translations.push_back(toVec3f(lt.translation));
        channel.rotations.push_back(toQuatf(lt.rotation));
        channel.scales.push_back(toVec3f(lt.scale));

        ufbx_free_scene(evalScene);
      }

      // Blend shape weights for this node's mesh
      bool hasMorphs = false;
      if (node->mesh) {
        ufbx_mesh* mesh = node->mesh;
        for (size_t di = 0; di < mesh->blend_deformers.count; di++) {
          ufbx_blend_deformer* blend = mesh->blend_deformers.data[di];
          for (size_t ci = 0; ci < blend->channels.count; ci++) {
            ufbx_blend_channel* blendChannel = blend->channels.data[ci];

            for (int fi = 0; fi < numFrames; fi++) {
              double evalTime = timeBegin + (double)fi * frameDuration;
              if (evalTime > timeEnd) evalTime = timeEnd;

              // Evaluate the blend channel weight at this time
              ufbx_prop weightProp = ufbx_evaluate_prop(
                  animStack->anim,
                  &blendChannel->element,
                  "DeformPercent",
                  evalTime);

              float weight = 0.0f;
              if (!(weightProp.flags & UFBX_PROP_FLAG_NOT_FOUND)) {
                weight = (float)(weightProp.value_vec3.x / 100.0);
              } else {
                weight = (float)(blendChannel->weight);
              }
              channel.weights.push_back(weight);
              if (fabs(weight) > epsilon) hasMorphs = true;
            }
          }
        }
      }

      if (hasTranslation || hasRotation || hasScale || hasMorphs) {
        if (!hasTranslation) channel.translations.clear();
        if (!hasRotation) channel.rotations.clear();
        if (!hasScale) channel.scales.clear();
        if (!hasMorphs) channel.weights.clear();

        animation.channels.emplace_back(channel);

        totalSizeInBytes += channel.translations.size() * sizeof(channel.translations[0]) +
            channel.rotations.size() * sizeof(channel.rotations[0]) +
            channel.scales.size() * sizeof(channel.scales[0]) +
            channel.weights.size() * sizeof(channel.weights[0]);
      }

      if (verboseOutput) {
        fmt::printf(
            "\ranimation %zu: %s (%d%%)",
            animIx,
            animName.c_str(),
            (int)(ni * 100 / scene->nodes.count));
      }
    }

    raw.AddAnimation(animation);

    if (verboseOutput) {
      fmt::printf(
          "\ranimation %zu: %s (%zu channels, %3.1f MB)\n",
          animIx,
          animName.c_str(),
          animation.channels.size(),
          (float)totalSizeInBytes * 1e-6f);
    }
  }
}

// -------------------------------------------------------
// Main entry point
// -------------------------------------------------------
bool LoadFBXFile(
    RawModel& raw,
    const std::string fbxFileName,
    const std::set<std::string>& textureExtensions,
    const GltfOptions& options) {
  // Load FBX file with ufbx
  ufbx_load_opts ufbx_opts = {};
  ufbx_opts.target_axes = ufbx_axes_right_handed_y_up;
  ufbx_opts.target_unit_meters = 1.0f;
  ufbx_opts.space_conversion = UFBX_SPACE_CONVERSION_TRANSFORM_ROOT;
  ufbx_opts.generate_missing_normals = true;

  ufbx_error error;
  ufbx_scene* scene = ufbx_load_file(fbxFileName.c_str(), &ufbx_opts, &error);

  if (!scene) {
    char error_buf[512];
    ufbx_format_error(error_buf, sizeof(error_buf), &error);
    fmt::fprintf(stderr, "ERROR: Failed to load FBX: %s\n  %s\n", fbxFileName, error_buf);
    return false;
  }

  if (verboseOutput) {
    fmt::printf(
        "Loaded FBX: %zu nodes, %zu meshes, %zu materials, %zu animations\n",
        scene->nodes.count,
        scene->meshes.count,
        scene->materials.count,
        scene->anim_stacks.count);
  }

  // Get the folder containing the FBX file for texture search
  namespace fs = std::filesystem;
  std::string fbxFolder = fs::path(fbxFileName).parent_path().string();
  if (fbxFolder.empty()) fbxFolder = ".";

  // Phase 1: Read node hierarchy (transforms + parent/child)
  ReadNodeHierarchy(raw, scene, scene->root_node, 0, "");

  // Phase 2: Read node attributes (meshes, cameras, lights)
  ReadNodeAttributes(raw, scene, scene->root_node, fbxFolder, textureExtensions);

  // Phase 3: Read animations
  ReadAnimations(raw, scene, options);

  ufbx_free_scene(scene);

  return true;
}
