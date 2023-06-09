#pragma once

#include "ExaStitchModel.h"

namespace exa {

  // reuse stitcher model for quick clusters
  struct QuickClustersModel : Model {
    typedef std::shared_ptr<QuickClustersModel> SP;

    static QuickClustersModel::SP load(const std::string umeshFileName)
    {
      auto m = ExaStitchModel::load(umeshFileName, "", "");

      QuickClustersModel::SP result = std::make_shared<QuickClustersModel>();
      (Model&)(*result) = (const Model&)(*m);
      result->indices = std::move(m->indices);
      result->vertices = std::move(m->vertices);
      return result;
    }

    std::vector<int> indices;
    std::vector<vec4f> vertices;

    // Statistics
    size_t numScalarsTotal;
    size_t numEmptyScalars;
    void memStats(size_t &elemVertexBytes,
                  size_t &elemIndexBytes,
                  size_t &gridletBytes,
                  size_t &emptyScalarsBytes,
                  size_t &nonEmptyScalarsBytes);
  };

} // ::exa
