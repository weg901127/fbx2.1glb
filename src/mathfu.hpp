/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once

#include <vector>

#include <mathfu/matrix.h>
#include <mathfu/quaternion.h>
#include <mathfu/rect.h>
#include <mathfu/vector.h>

/**
 * All the mathfu:: implementations of our core data types.
 */

template <class T, int d>
struct Bounds {
  mathfu::Vector<T, d> min;
  mathfu::Vector<T, d> max;
  bool initialized = false;

  void Clear() {
    min = mathfu::Vector<T, d>();
    max = mathfu::Vector<T, d>();
    initialized = false;
  }

  void AddPoint(const mathfu::Vector<T, d>& p) {
    if (initialized) {
      for (int ii = 0; ii < d; ii++) {
        min(ii) = std::min(min(ii), p(ii));
        max(ii) = std::max(max(ii), p(ii));
      }
    } else {
      min = p;
      max = p;
      initialized = true;
    }
  }
};

typedef mathfu::Vector<uint16_t, 4> Vec4i;
typedef mathfu::Matrix<uint16_t, 4> Mat4i;
typedef mathfu::Vector<float, 2> Vec2f;
typedef mathfu::Vector<float, 3> Vec3f;
typedef mathfu::Vector<float, 4> Vec4f;
typedef mathfu::Matrix<float, 2> Mat2f;
typedef mathfu::Matrix<float, 3> Mat3f;
typedef mathfu::Matrix<float, 4> Mat4f;
typedef mathfu::Quaternion<float> Quatf;
typedef Bounds<float, 3> Boundsf;

#define VEC3F_ONE (Vec3f{1.0f})
#define VEC3F_ZERO (Vec3f{0.0f})
#define VEC4F_ONE (Vec4f{1.0f})
#define VEC4F_ZERO (Vec4f{0.0f})

template <class T, int d>
inline std::vector<T> toStdVec(const mathfu::Vector<T, d>& vec) {
  std::vector<T> result(d);
  for (int ii = 0; ii < d; ii++) {
    result[ii] = vec[ii];
  }
  return result;
}

template <class T>
std::vector<T> toStdVec(const mathfu::Quaternion<T>& quat) {
  return std::vector<T>{quat.vector()[0], quat.vector()[1], quat.vector()[2], quat.scalar()};
}

// ufbx conversion helpers (used only in src/fbx/)
// Defined inline here for convenience; ufbx.h is lightweight.
#if defined(UFBX_UFBX_H_INCLUDED)

inline Vec3f toVec3f(const ufbx_vec3& v) {
  return Vec3f((float)v.x, (float)v.y, (float)v.z);
}

inline Vec4f toVec4f(const ufbx_vec4& v) {
  return Vec4f((float)v.x, (float)v.y, (float)v.z, (float)v.w);
}

inline Quatf toQuatf(const ufbx_quat& q) {
  return Quatf((float)q.w, (float)q.x, (float)q.y, (float)q.z);
}

inline Mat4f toMat4f(const ufbx_matrix& m) {
  // ufbx_matrix is 3 column vectors + translation (affine 3x4)
  // Expand to 4x4 column-major for mathfu
  auto result = Mat4f::Identity();
  result(0, 0) = (float)m.cols[0].x; result(1, 0) = (float)m.cols[0].y; result(2, 0) = (float)m.cols[0].z;
  result(0, 1) = (float)m.cols[1].x; result(1, 1) = (float)m.cols[1].y; result(2, 1) = (float)m.cols[1].z;
  result(0, 2) = (float)m.cols[2].x; result(1, 2) = (float)m.cols[2].y; result(2, 2) = (float)m.cols[2].z;
  result(0, 3) = (float)m.cols[3].x; result(1, 3) = (float)m.cols[3].y; result(2, 3) = (float)m.cols[3].z;
  return result;
}

#endif // UFBX_UFBX_H_INCLUDED
