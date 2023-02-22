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

#include <fstream>
#include "model/ExaBrickModel.h"
#include "common.h"
#include "YueKDTree.h"
#include "YueVolume.h"

/* tool to build a TF-dependent, majorant-optimized
  kd-tree given an *ExaBricks* model */
namespace exa {

  struct {
    std::string scalarFileName = "";
    std::string exaBrickFileName = "";
    std::string xfFileName = "";
    std::string outFileName = "majorants.bin";
    int numLeaves = 100;
  } cmdline;

  extern "C" int main(int argc, char** argv)
  {
    for (int i=1;i<argc;i++) {
      const std::string arg = argv[i];

      if (arg == "-scalar" || arg == "-scalars") {
        cmdline.scalarFileName = argv[++i];
      }
      else if (arg == "-bricks") {
        cmdline.exaBrickFileName = argv[++i];
      }
      else if (arg == "-xf") {
        cmdline.xfFileName = argv[++i];
      }
      else if (arg == "-o") {
        cmdline.outFileName = argv[++i];
      }
      else if (arg == "-n") {
        cmdline.numLeaves = std::atoi(argv[++i]);
      }
    }

    if (cmdline.scalarFileName.empty()) {
      throw std::runtime_error("No scalar file given");
    }

    if (cmdline.exaBrickFileName.empty()) {
      throw std::runtime_error("No exabrick file given");
    }

    if (cmdline.xfFileName.empty()) {
      throw std::runtime_error("No transfer function file given");
    }

    ExaBrickModel::SP model = ExaBrickModel::load(cmdline.exaBrickFileName,
                                                  cmdline.scalarFileName,
                                                  ""/*kdtree file, empty*/);

    if (!model) {
      throw std::runtime_error("Could not load exabrick model");
    }

    // Load TF
    std::ifstream xfFile(cmdline.xfFileName, std::ios::binary);

    if (!xfFile.good()) {
      throw std::runtime_error("Could not open TF");
    }

    static const size_t xfFileFormatMagic = 0x1235abc000;
    size_t magic;
    xfFile.read((char*)&magic,sizeof(xfFileFormatMagic));
    if (magic != xfFileFormatMagic) {
      throw std::runtime_error("Not a valid TF file");
    }

    float opacityScale;
    xfFile.read((char*)&opacityScale,sizeof(opacityScale));

    range1f absDomain;
    xfFile.read((char*)&absDomain.lower,sizeof(absDomain.lower));
    xfFile.read((char*)&absDomain.upper,sizeof(absDomain.upper));

    range1f relDomain;
    xfFile.read((char*)&relDomain,sizeof(relDomain));

    std::vector<float> colorMap;
    int numColorMapValues;
    xfFile.read((char*)&numColorMapValues,sizeof(numColorMapValues));
    colorMap.resize(numColorMapValues*4);
    xfFile.read((char*)colorMap.data(),colorMap.size()*sizeof(colorMap[0]));

    std::cout << "=== Color Map ===\n";
    std::cout << "ABS range: " << absDomain << '\n';
    std::cout << "REL range: " << relDomain << '\n';
    std::cout << "Color map: " << numColorMapValues << " values\n";

    std::cout << "Models value range is: " << model->valueRange << '\n';

    // this happens twice in viewer.cpp, so we just do this here, too
    // (and a 2nd time in the c'tor of YueVolue...)
    range1f r{
      model->valueRange.lower + (relDomain.lower/100.f) * (model->valueRange.upper-model->valueRange.lower),
      model->valueRange.lower + (relDomain.upper/100.f) * (model->valueRange.upper-model->valueRange.lower),
    };

    YueVolume vol(model,&colorMap,r,relDomain);
    volkd::KDTree kdtree(vol,&colorMap,cmdline.numLeaves);

    std::vector<std::pair<box3f,float>> domains;
    while (!kdtree.nodes.empty()) {
      volkd::Node node = kdtree.nodes.top();
      kdtree.nodes.pop();
      range1f tfRange = vol.min_max(node.domain,&colorMap);
      std::cout << "Domain " << domains.size() << ": "
                << node.domain << ", majorant: " << tfRange.upper << '\n';
      domains.push_back({node.domain,tfRange.upper});
    }

    std::ofstream domainsFile(cmdline.outFileName, std::ios::binary);
    uint64_t numDomains = domains.size();
    domainsFile.write((char *)&numDomains,sizeof(numDomains));
    domainsFile.write((char *)domains.data(),domains.size()*sizeof(domains[0]));
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

