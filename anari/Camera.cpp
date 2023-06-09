#include "Camera.h"
#include <OWLRenderer.h>
using namespace anari;
using namespace owl;

namespace exa {

Camera *Camera::createInstance(std::string_view subtype, ExaStitchGlobalState *d)
{
  if (subtype=="perspective")
    return new Camera(d);
  else {
    std::cerr << "ANARICamera subtype: " << subtype << " not supported\n";
    return (Camera *)new Object(d);
  }
}

Camera::Camera(ExaStitchGlobalState *s)
  : Object(s)
{}

void Camera::commit()
{
  pos = getParam<anari::float3>("position", anari::float3(0.f));
  dir = normalize(getParam<anari::float3>("direction", anari::float3(0.f, 0.f, 1.f)));
  up = normalize(getParam<anari::float3>("up", anari::float3(0.f, 1.f, 0.f)));

  float fovy = getParam<float>("fovy", 60.f*M_PI/180.f);
  float aspect = getParam<float>("aspect", 1.f);
  //std::cout << "Camera::commit() " <<pos << ',' << dir << ',' << up << ',' << fovy << ',' << aspect << '\n';

  anari::float2 imgPlaneSize;
  imgPlaneSize.y = 2.f * tanf(0.5f * fovy);
  imgPlaneSize.x = imgPlaneSize.y * aspect;

  dir_du = normalize(cross(dir, up)) * imgPlaneSize.x;
  dir_dv = normalize(cross(dir_du, dir)) * imgPlaneSize.y;
  dir_00 = dir - .5f * dir_du - .5f * dir_dv;

  if (!deviceState()->owlRenderer) {
    return;
  }

  apply();

  deviceState()->owlRenderer->resetAccum();

  markUpdated();
}

void Camera::apply()
{
  if (!deviceState()->owlRenderer) {
    return;
  }

  vec2i fbSize = deviceState()->owlRenderer->fbSize;
  deviceState()->owlRenderer->setCamera({pos.x,pos.y,pos.z},
                                        {dir_00.x,dir_00.y,dir_00.z},
                                        vec3f(dir_du.x,dir_du.y,dir_du.z)/(float)fbSize.x,
                                        vec3f(dir_dv.x,dir_dv.y,dir_dv.z)/(float)fbSize.y);

}

} // ::exa

EXA_ANARI_TYPEFOR_DEFINITION(exa::Camera *);

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

