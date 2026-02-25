/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Modified: FBX SDK replaced with ufbx (MIT/Public Domain)
 */

#pragma once

#include "raw/RawModel.hpp"

bool LoadFBXFile(
    RawModel& raw,
    const std::string fbxFileName,
    const std::set<std::string>& textureExtensions,
    const GltfOptions& options);
