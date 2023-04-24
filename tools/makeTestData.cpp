#include <fstream>
#include <vector>
#include "model/ABRs.h"

using namespace exa;

// (currently) this tool just generates *hardcoded* mini
// test data sets!
int main(int argc, char **argv) {
  std::vector<ExaBrick> bricks(2);
  bricks[0].lower = {0,0,0};
  bricks[0].size = {1,1,1};
  bricks[0].level = 1;
  bricks[0].begin = 0;

  bricks[1].lower = {2,0,0};
  bricks[1].size = {2,2,2};
  bricks[1].level = 0;
  bricks[1].begin = 1;

  std::vector<int> indices;
  for (int i=0; i<9; ++i) indices.push_back(i);

  std::ofstream bricksFile("exa-test-data.bricks");
  for (int i=0; i<bricks.size(); ++i) {
    bricksFile.write((const char *)&bricks[i].size, sizeof(bricks[i].size));
    bricksFile.write((const char *)&bricks[i].lower, sizeof(bricks[i].lower));
    bricksFile.write((const char *)&bricks[i].level, sizeof(bricks[i].level));
    bricksFile.write((const char *)(indices.data()+bricks[i].begin),
                     owl::volume(bricks[i].size)*sizeof(indices[0]));
  }

  std::vector<float> scalars(9);
  scalars[0] = 0.5f;

  scalars[1] = scalars[3] = scalars[5] = scalars[7] = 0.f;
  scalars[2] = scalars[4] = scalars[6] = scalars[8] = 1.f;

  std::ofstream scalarFile0("exa-test-data.scalar0");
  scalarFile0.write((const char *)scalars.data(), scalars.size()*sizeof(scalars[0]));

  for (size_t i=0; i<scalars.size(); ++i) {
    scalars[i] = fmaxf(0.f,scalars[i]-0.2f);
  }

  std::ofstream scalarFile1("exa-test-data.scalar1");
  scalarFile1.write((const char *)scalars.data(), scalars.size()*sizeof(scalars[0]));
}
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

