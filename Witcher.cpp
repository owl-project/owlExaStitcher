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

#include "Witcher.h"
namespace exa {

  /*! query if one of the cmaeke options defined for the witcher library
    are set, and if so, check return their value */
  bool hasOption(std::string option, int *value)
  {
    if (option == "EXA_STITCH_WITH_EXA_STITCH_SAMPLER") {
#ifdef EXA_STITCH_WITH_EXA_STITCH_SAMPLER
      *value = EXA_STITCH_WITH_EXA_STITCH_SAMPLER;
      return true;
#endif
    }
    else if (option == "EXA_STITCH_WITH_EXA_BRICK_SAMPLER") {
#ifdef EXA_STITCH_WITH_EXA_BRICK_SAMPLER
      *value = EXA_STITCH_WITH_EXA_BRICK_SAMPLER;
      return true;
#endif
    }
    else if (option == "EXA_STITCH_WITH_AMR_CELL_SAMPLER") {
#ifdef EXA_STITCH_WITH_AMR_CELL_SAMPLER
      *value = EXA_STITCH_WITH_AMR_CELL_SAMPLER;
      return true;
#endif
    }
    else if (option == "EXA_STITCH_MIRROR_EXAJET") {
#ifdef EXA_STITCH_MIRROR_EXAJET
      *value = EXA_STITCH_MIRROR_EXAJET;
      return true;
#endif
    }
    else if (option == "EXA_STITCH_EXA_BRICK_SAMPLER_MODE") {
#ifdef EXA_STITCH_EXA_BRICK_SAMPLER_MODE
      *value = EXA_STITCH_EXA_BRICK_SAMPLER_MODE;
      return true;
#endif
    }
    else if (option == "EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE") {
#ifdef EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE
      *value = EXA_STITCH_EXA_BRICK_TRAVERSAL_MODE;
      return true;
#endif
    }

    return false;
  }
  
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

