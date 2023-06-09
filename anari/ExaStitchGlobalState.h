// ======================================================================== //
// Copyright 2022-2023 Stefan Zellmann                                      //
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

// helium
#include "helium/BaseGlobalDeviceState.h"

namespace exa {

struct OWLRenderer;
struct Frame;

struct ExaStitchGlobalState : public helium::BaseGlobalDeviceState
{
  struct ObjectCounts
  {
    size_t frames{0};
    size_t cameras{0};
    size_t renderers{0};
    size_t worlds{0};
    size_t instances{0};
    size_t groups{0};
    size_t surfaces{0};
    size_t geometries{0};
    size_t materials{0};
    size_t arrays{0};
    size_t unknown{0};
  } objectCounts;

  std::shared_ptr<OWLRenderer> owlRenderer{ nullptr };

  Frame *currentFrame{nullptr};

  ExaStitchGlobalState(ANARIDevice d) : helium::BaseGlobalDeviceState(d) {}
};

} // ::exa
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

