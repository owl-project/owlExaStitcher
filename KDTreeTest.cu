#include <cuda_runtime.h>
#include <vector>
#include <owl/owl.h>
#include <owl/common/math/box.h>
#include "KDTree.cuh" // include on device
#include "KDTree.h"   // include on host

using namespace exa;
using namespace owl;

struct PRD {

};

__device__
void isect(const owl::Ray &ray, PRD &prd, int primID, float tmin, float tmax, KDTreeHitRec &hitRec)
{
  printf("%i, (%f,%f)\n",primID,tmin,tmax);

  hitRec.hit = true;
  hitRec.t   = tmin;
}

__global__ void test(KDTreeTraversable tree)
{
  owl::Ray ray;
  ray.origin = vec3f(-1.f,-1.f,0.f);
  ray.direction = normalize(vec3f(0.5f,1,0.1f));

  PRD prd;

  kd::traceRay(tree,ray,prd,isect);
}

int main() {
  std::vector<box3f> boxes;
  boxes.push_back(box3f({0,0,0},{2,2,2}));
  boxes.push_back(box3f({0,2,0},{1,3,2}));
  boxes.push_back(box3f({1,2,0},{2,3,2}));

  auto kd = KDTree::build(boxes.size(),boxes.data());

  kd->initGPU();

  test<<<1,1>>>(kd->deviceTraversable);
  cudaDeviceSynchronize();
}

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

