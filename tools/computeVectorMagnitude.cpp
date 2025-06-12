
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

static std::string g_outFileName;
static std::string g_inFileNames[3];
static int g_inFileCount{0};

inline
std::vector<float> fromFile(std::string fn) {
  std::vector<float> res;
  std::ifstream scalarFile(fn, std::ios::binary | std::ios::ate);
  size_t numBytes = scalarFile.tellg();
  scalarFile.close();
  scalarFile.open(fn, std::ios::binary);
  if (!scalarFile.good()) return res;
  res.resize(numBytes/sizeof(float));
  scalarFile.read((char *)res.data(),numBytes);
  return res;
}

int main(int argc, char *argv[]) {

  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    if (arg == "-o")
      g_outFileName = argv[++i];
    else if (arg[0] != '-')
      g_inFileNames[g_inFileCount++] = arg;
  }

  if (g_outFileName.empty() || g_inFileCount != 3) {
    std::cerr << "Usage: ./app vector_x.bin vector_y.bin vector_z.bin -o output.bin" << std::endl;
    exit(0);
  }

  auto X = fromFile(g_inFileNames[0]);
  auto Y = fromFile(g_inFileNames[1]);
  auto Z = fromFile(g_inFileNames[2]);

  assert(X.size() == Y.size() && X.size() == Z.size());

  std::vector<float> res(X.size());

  for (size_t i=0; i<X.size(); ++i) {
    res[i] = sqrtf(X[i]*X[i]+Y[i]*Y[i]+Z[i]*Z[i]);
  }

  std::ofstream outfile(g_outFileName, std::ios::binary);
  size_t numBytes = res.size()*sizeof(res[0]);
  outfile.write((const char *)res.data(),numBytes);
}




