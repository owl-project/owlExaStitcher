// ======================================================================== //
// Copyright 2018-2021 Ingo Wald                                            //
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

#include "umesh/UMesh.h"
#include "umesh/io/IO.h"
#include "umesh/check.h"
// #include "tetty/UMesh.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include <set>
#include <map>
#include <fstream>
#include <atomic>
#include <array>
#include <cstring>


#ifndef PRINT
# define PRINT(var) std::cout << #var << "=" << var << std::endl;
#ifdef __WIN32__
# define PING std::cout << __FILE__ << "::" << __LINE__ << ": " << __FUNCTION__ << std::endl;
#else
# define PING std::cout << __FILE__ << "::" << __LINE__ << ": " << __PRETTY_FUNCTION__ << std::endl;
#endif
#endif


using namespace umesh;

int macroCellWidth = 8;

struct Cube {
  vec3f lower;
  int   level;
  std::array<int,8> scalarIDs;
};

vec3i make_vec3i(vec3f v) { return { int(v.x), int(v.y), int(v.z) }; }
vec3f make_vec3f(vec3i v) { return { float(v.x), float(v.y), float(v.z) }; }

vec3i cellID(const Cube &cube)
{
  vec3i cid = make_vec3i(cube.lower);
  if (cube.lower.x < 0.f) cid.x -= ((1<<cube.level)-1);
  if (cube.lower.y < 0.f) cid.y -= ((1<<cube.level)-1);
  if (cube.lower.z < 0.f) cid.z -= ((1<<cube.level)-1);
  cid = cid / (1<<cube.level);

  // if (cid == vec3i(-1,-1,-1)) {
  //   PING;
  //   PRINT(cube.lower);
  //   PRINT(cid);
  // }
  return cid;
}

box3i cellBounds(const Cube &cube)
{
  vec3i cell = cellID(cube);
  return { cell, cell+vec3i(1) };
}

vec3i mcID(const Cube &cube)
{
  vec3i cid = cellID(cube);
  if (cid.x < 0) cid.x -= (macroCellWidth-1);
  if (cid.y < 0) cid.y -= (macroCellWidth-1);
  if (cid.z < 0) cid.z -= (macroCellWidth-1);
  vec3i mcid = cid / macroCellWidth;
  // if (mcid == vec3i(-1,-1,-1)) {
  //   PING;
  //   PRINT(cube.lower);
  //   PRINT(cellID(cube));
  //   PRINT(mcid);
  // }
  return mcid;
}


struct Brick {
  void create(const box3i &bounds)
  {
    lower    = bounds.lower;
    dbg_bounds = bounds;
    if (lower.x < -1000000) {
      PING; PRINT(bounds);
    }
    numCubes = bounds.size();
    int numScalars
      = (numCubes.x+1)
      * (numCubes.y+1)
      * (numCubes.z+1);
    scalarIDs.resize(numScalars);
    std::fill(scalarIDs.begin(),scalarIDs.end(),-1);
  }
  box3i dbg_bounds;

  void write(vec3i localVertex, int scalarID, bool dbg=false)
  {
    int idx
      = localVertex.x + (numCubes.x+1)*(localVertex.y + (numCubes.y+1)*localVertex.z);
    if (idx < 0 || idx >= scalarIDs.size()) {
      PRINT(localVertex);
      PRINT(numCubes);
      PRINT(idx);
      throw std::runtime_error("invalid local vertex index");
    }
    if (scalarIDs[idx] != -1 && scalarIDs[idx] != scalarID) {
      PING;
      PRINT(scalarIDs[idx]);
      PRINT(scalarID);
      throw std::runtime_error("invalid local write");
    }
    if (dbg) std::cout << " -> writing to " << localVertex << " (@ " << idx << ") = " << scalarID << std::endl;
    scalarIDs[idx] = scalarID;
  }

  void write(const Cube &cube)
  {
    vec3i mcid = mcID(cube);
    bool dbg = (mcid == vec3i(0,-1,0));
    if (dbg) {
      std::cout << "----------" << std::endl;
    PING;
    PRINT(cube.lower);
    PRINT(cube.level);
    PRINT(cellID(cube));
    PRINT(mcID(cube));
    PRINT(lower);
    PRINT(dbg_bounds);
    PRINT(numCubes);
    for (int i=0;i<8;i++)
      std::cout << "  scalars[" << i << "] = " << cube.scalarIDs[i] << std::endl;
    }
            // v[0] = vertex[0][0][0];
            // v[1] = vertex[0][0][1];
            // v[2] = vertex[0][1][1];
            // v[3] = vertex[0][1][0];
            // v[4] = vertex[1][0][0];
            // v[5] = vertex[1][0][1];
            // v[6] = vertex[1][1][1];
            // v[7] = vertex[1][1][0];
    int vtkOrder[8] = { 0,1,3,2,4,5,7,6 };
    vec3i base = cellID(cube) - this->lower;
    if (dbg) PRINT(base);
    for (int iz=0;iz<2;iz++)
      for (int iy=0;iy<2;iy++)
        for (int ix=0;ix<2;ix++) {
          if (dbg) PRINT(vec3i(ix,iy,iz));
          write(base+vec3i(ix,iy,iz),cube.scalarIDs[vtkOrder[4*iz+2*iy+ix]],dbg);
        }
  }


  vec3i lower;
  int   level;
  
  vec3i numCubes;
  std::vector<int> scalarIDs;
};

box3f worldBounds(const Brick &brick)
{
  box3f bb;
  bb.lower = make_vec3f(brick.lower * (1<<brick.level));
  bb.upper = bb.lower + make_vec3f(brick.numCubes * (1<<brick.level));
  return bb;
}
  
/*! the 'cells' are all in a space where each cell is exactly 1
    int-coord wide, so the second cell on level 1 is _not_ at
    (2,2,2)-(4,4,4), but at (1,1,1)-(2,2,2). To translate from this
    level-L cell space to world coordinates, take cell (i,j,k) and get
    lower=((i,j,k)+.5f)*(1<<L), and upper = lower+(1<<L) */
std::map<vec3i,Brick> makeBricksForLevel(int level,
                                         std::vector<Cube> &cubes)
{
  std::map<vec3i,box3i> mcBounds;
  for (auto cube : cubes) {
    mcBounds[mcID(cube)].extend(cellBounds(cube));
  }

  std::map<vec3i,Brick> mcBricks;
  for (auto &mc : mcBounds) {
    mcBricks[mc.first].create(mc.second);
    mcBricks[mc.first].level = level;
  }

  for (auto cube : cubes) {
    mcBricks[mcID(cube)].write(cube);
  }
  return mcBricks;
}

void writeQuadOBJ(std::ostream &out,
                  vec3f base,
                  vec3f du,
                  vec3f dv)
{
  vec3f v00 = base;
  vec3f v01 = base+du;
  vec3f v11 = base+du+dv;
  vec3f v10 = base+dv;
  out << "v " << v00.x << " " << v00.y << " " << v00.z << std::endl;
  out << "v " << v01.x << " " << v01.y << " " << v01.z << std::endl;
  out << "v " << v10.x << " " << v10.y << " " << v10.z << std::endl;
  out << "v " << v11.x << " " << v11.y << " " << v11.z << std::endl;
  out << "f -1 -2 -4 -3" << std::endl;
}

void writeOBJ(std::ostream &out, const box3f &box)
{
  vec3f dx(box.size().x,0.f,0.f);
  vec3f dy(0.f,box.size().y,0.f);
  vec3f dz(0.f,0.f,box.size().z);
  writeQuadOBJ(out,box.lower,dx,dy);
  writeQuadOBJ(out,box.lower,dx,dz);
  writeQuadOBJ(out,box.lower,dy,dz);
  writeQuadOBJ(out,box.upper,-dx,-dy);
  writeQuadOBJ(out,box.upper,-dx,-dz);
  writeQuadOBJ(out,box.upper,-dy,-dz);
}

void writeBIN(std::ostream &out, const Brick &brick)
{
  out.write((const char *)&brick.lower,sizeof(brick.lower));
  out.write((const char *)&brick.level,sizeof(brick.level));
  out.write((const char *)&brick.numCubes,sizeof(brick.numCubes));
  out.write((const char *)brick.scalarIDs.data(),brick.scalarIDs.size()*sizeof(brick.scalarIDs[0]));
}

inline double getCurrentTime()
{
#ifdef _WIN32
  SYSTEMTIME tp; GetSystemTime(&tp);
  /*
     Please note: we are not handling the "leap year" issue.
 */
  size_t numSecsSince2020
      = tp.wSecond
      + (60ull) * tp.wMinute
      + (60ull * 60ull) * tp.wHour
      + (60ull * 60ul * 24ull) * tp.wDay
      + (60ull * 60ul * 24ull * 365ull) * (tp.wYear - 2020);
  return double(numSecsSince2020 + tp.wMilliseconds * 1e-3);
#else
  struct timeval tp; gettimeofday(&tp,nullptr);
  return double(tp.tv_sec) + double(tp.tv_usec)/1E6;
#endif
}

void makeGridsFor(const std::string &fileName)
{
  std::cout << "==================================================================" << std::endl;
  std::cout << "making grids for " << fileName << std::endl;
  std::cout << "==================================================================" << std::endl;
  const char *ext = strstr(fileName.c_str(),"_");
  if (!ext)
    throw std::runtime_error("'"+fileName+"' is not a cubes file!?");
  while (const char *next = strstr(ext+1,"_"))
    ext = next;
  int level;
  int rc = sscanf(ext,"_%i.cubes",&level);
  if (rc != 1) 
    throw std::runtime_error("'"+fileName+"' is not a cubes file!?");

  double t_first = getCurrentTime();
  std::vector<Cube> cubes;
  std::ifstream in(fileName,std::ios::binary);
  while (!in.eof()) {
    Cube cube;
    in.read((char*)&cube,sizeof(cube));
    cubes.push_back(cube);
  }
  std::map<vec3i,Brick> bricks
    = makeBricksForLevel(level,cubes);
  double t_last = getCurrentTime();
  std::cout << t_last-t_first << '\n';

#if 1
 int numBricksGenerated = 0;
 int numCubesInBricks = 0;
 int numScalarsInBricks = 0;
  for (auto &brick : bricks) {
    numBricksGenerated++;
    numCubesInBricks += brick.second.numCubes.x*brick.second.numCubes.y*brick.second.numCubes.z;
    numScalarsInBricks += brick.second.scalarIDs.size();
  }
  PRINT(numBricksGenerated);
  PRINT(numCubesInBricks);
  PRINT(numScalarsInBricks);
  static int totalBricksGenerated = 0;
  static int totalCubesInBricks = 0;
  static int totalScalarsInBricks = 0;

  totalBricksGenerated += numBricksGenerated;
  totalCubesInBricks  += numCubesInBricks;
  totalScalarsInBricks += numScalarsInBricks;
  
  PRINT(totalBricksGenerated);
  PRINT(totalCubesInBricks);
  PRINT(totalScalarsInBricks);
  PRINT(prettyNumber(totalBricksGenerated));
  PRINT(prettyNumber(totalCubesInBricks));
  PRINT(prettyNumber(totalScalarsInBricks));
  static int fileID=0;
  std::ofstream out;
  if (fileID++ == 0)
    out.open("/tmp/out.grids", std::ios_base::binary);
  else
    out.open("/tmp/out.grids", std::ios_base::binary|std::ios_base::app);
  for (auto &brick : bricks) {
    writeBIN(out,brick.second);
  }
#else
  std::ofstream out("/tmp/out.obj");
  for (auto &brick : bricks) {
    writeOBJ(out,worldBounds(brick.second));
  }
#endif
}

int main(int ac, char **av)
{
  for (int i=1;i<ac;i++)
    makeGridsFor(av[i]);
}
