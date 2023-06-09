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

#include <fstream>
#include "AMRCellModel.h"

namespace exa {


  AMRCellModel::SP AMRCellModel::load(const std::string cellFileName,
                                      const std::string scalarFileName)
  {
    AMRCellModel::SP result = std::make_shared<AMRCellModel>();

    std::vector<AMRCell> &cells = result->cells;
    std::vector<float> &scalars = result->scalars;
    box3f &cellBounds           = result->cellBounds;
    range1f &valueRange         = result->valueRange;

    std::ifstream scalarFile(scalarFileName, std::ios::binary | std::ios::ate);
    if (scalarFile.good()) {
      size_t numBytes = scalarFile.tellg();
      scalarFile.close();
      scalarFile.open(scalarFileName, std::ios::binary);
      if (scalarFile.good()) {
        scalars.resize(numBytes/sizeof(float));
        scalarFile.read((char *)scalars.data(),scalars.size()*sizeof(float));
      }
    }

    // ==================================================================
    // AMR cells
    // ==================================================================

    std::ifstream amrCellFile(cellFileName, std::ios::binary | std::ios::ate);
    if (amrCellFile.good()) {
      size_t numBytes = amrCellFile.tellg();
      amrCellFile.close();
      amrCellFile.open(cellFileName, std::ios::binary);
      if (amrCellFile.good()) {
        cells.resize(numBytes/sizeof(AMRCell));
        amrCellFile.read((char *)cells.data(),cells.size()*sizeof(AMRCell));
      }
    }

    for (size_t i=0; i<cells.size(); ++i) {
      box3f bounds(vec3f(cells[i].pos),
                   vec3f(cells[i].pos+vec3i(1<<cells[i].level)));
      cellBounds.extend(bounds.lower);
      cellBounds.extend(bounds.upper);
      if (i < scalars.size())
        valueRange.extend(scalars[i]);
    }

    return result; 
  }

  void AMRCellModel::memStats(size_t &cellsBytes, size_t &scalarsBytes)
  {
    cellsBytes = cells.empty()   ? 0 : cells.size()*sizeof(cells[0]);
    scalarsBytes = cells.empty() ? 0 : scalars.size()*sizeof(scalars[0]);
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

