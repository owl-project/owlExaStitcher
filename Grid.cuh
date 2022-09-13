// ======================================================================== //
// Copyright 2022-2022 Stefan Zellmann                                      //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

namespace exa {

  __device__ inline
  size_t linearIndex(const owl::vec3i index, const owl::vec3i dims)
  {
    return index.z * size_t(dims.x) * dims.y
         + index.y * dims.x
         + index.x;
  }

  __device__ inline
  owl::vec3i gridIndex(const size_t index, const owl::vec3i grid)
  {
    const auto stride_y = (size_t)grid.x;
    const auto stride_z = (size_t)grid.y * (size_t)grid.x;
    return  owl::vec3i(index % stride_y, (index % stride_z) / stride_y, index / stride_z);
  }

  __device__ inline
  vec3i projectOnGrid(const owl::vec3f V,
                      const owl::vec3i dims,
                      const owl::box3f worldBounds)
  {
    const vec3f V01 = (V-worldBounds.lower)/(worldBounds.upper-worldBounds.lower);
    return clamp(vec3i(V01*vec3f(dims)),vec3i(0),dims-vec3i(1));
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

