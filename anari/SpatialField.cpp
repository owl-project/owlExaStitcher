#include "array/Array3D.h"
#include "SpatialField.h"
#include <OWLRenderer.h>
#include "model/ExaBrickModel.h" // for struct ExaBrick

namespace exa {

SpatialField *SpatialField::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  if (subtype == "amr") {
    return new SpatialField(d);
  } else {
    std::cerr << "ANARISpatialField subtype: " << subtype << " not supported\n";
    return (SpatialField *)new Object(d);
  }
}

SpatialField::SpatialField(ExaStitchGlobalState *d)
  : Object(d)
{}

void SpatialField::commit()
{
  blockBounds = getParamObject<Array1D>("block.bounds");
  if (!blockBounds) {
    std::cerr << "SpatialField::commit(): no block.bounds provided\n";
  }

  blockLevel = getParamObject<Array1D>("block.level");
  if (!blockLevel) {
    std::cerr << "SpatialField::commit(): no block.level provided\n";
    return;
  }

  blockData = getParamObject<ObjectArray>("block.data");
  if (!blockData) {
    std::cerr << "SpatialField::commit(): no block.data provided\n";
    return;
  }

  // std::cout << blockBounds->totalSize() << '\n';
  // std::cout << blockLevel->totalSize() << '\n';
  // std::cout << blockData->totalSize() << '\n';

  std::vector<ExaBrick> bricks(blockBounds->totalSize());
  for (size_t i=0; i<bricks.size(); ++i) {
    box3i bound = *(blockBounds->beginAs<box3i>()+i);
    int level = *(blockLevel->beginAs<int>()+i);
    int cellWidth = (1<<level);
    bricks[i].lower.x = bound.lower.x*cellWidth;
    bricks[i].lower.y = bound.lower.y*cellWidth;
    bricks[i].lower.z = bound.lower.z*cellWidth;
    bricks[i].size.x = (bound.upper.x-bound.lower.x)+1;
    bricks[i].size.y = (bound.upper.y-bound.lower.y)+1;
    bricks[i].size.z = (bound.upper.z-bound.lower.z)+1;
    bricks[i].level = level;
    bricks[i].begin = 0;
    if (i>0) {
      bricks[i].begin = bricks[i-1].size.x*bricks[i-1].size.y*bricks[i-1].size.z;
    }
  }

  // convert to flat array
  size_t numCells = bricks.back().begin+volume(bricks.back().size);
  std::vector<float> scalars(numCells);
  for (size_t i=0; i<blockData->totalSize(); ++i) {
    const Array3D &bd = *((const Array3D *)blockData->handlesBegin()[i]);
    // assume stride=1..
    for (unsigned z=0;z<bd.size().z;++z) {
      for (unsigned y=0;y<bd.size().y;++y) {
        for (unsigned x=0;x<bd.size().x;++x) {
          size_t index = z*size_t(bd.size().x)*bd.size().y
                       + y*bd.size().x
                       + x;
          scalars[bricks[i].begin+index] = bd.dataAs<float>()[index];
        }
      }
    }
  }

  deviceState()->owlRenderer
    = std::make_shared<OWLRenderer>(bricks.data(),
                                    scalars.data(),
                                    bricks.size());

  markUpdated();
}

} // ::exa 

EXA_ANARI_TYPEFOR_DEFINITION(exa::SpatialField *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

