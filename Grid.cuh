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

namespace exa {

  __device__ inline
  size_t linearIndex(const owl::vec3i index, const owl::vec3i dims)
  {
    return index.z * size_t(dims.x) * dims.y
         + index.y * dims.x
         + index.x;
  }

  __device__ inline
  vec3i projectOnGrid(const owl::vec3f V,
                      const owl::vec3i dims,
                      const owl::box3f worldBounds)
  {
    const vec3f V01 = (V-worldBounds.lower)/(worldBounds.upper-worldBounds.lower);
    return vec3i(V01*vec3f(dims-1));
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

