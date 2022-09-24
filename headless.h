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

#pragma once

#include <string>
#include <QObject>
#include "owl/common/math/vec.h"
#include "qtOWL/Camera.h"

namespace exa {

  struct Headless : QObject {

    Headless(const std::string &title = "OWL Sample Viewer",
             const owl::vec2i &initWindowSize=owl::vec2i(1024,1024));

    ~Headless();

    void setOutFileName(std::string fileName);
    void setOutFileNameFPS(std::string fileName);

    void run();

    virtual void resize(const owl::vec2i &newSize);

    void setTitle(const std::string &s);

    virtual void render() {}

    virtual void draw() {}

    virtual void key(char key, const owl::vec2i &/*where*/) {}

    virtual void mouseButtonLeft(const owl::vec2i &where, bool pressed) {}
    virtual void mouseDragLeft(const owl::vec2i &where, const owl::vec2i &delta) {}

    virtual void cameraChanged() {}

    owl::vec2i getWindowSize() const { return fbSize; }

    qtOWL::Camera &getCamera() { return camera; }

    void screenShot(const std::string &fileName);

    virtual void setWorldScale(const float worldScale);

    /*! set a new window aspect ratio for the camera, update the
      camera, and notify the app */
    void setAspect(const float aspect)
    {
      camera.setAspect(aspect);
      updateCamera();
    }

    void updateCamera();

        /*! set a new orientation for the camera, update the camera, and
      notify the app */
    void setCameraOrientation(/* camera origin    : */const owl::vec3f &origin,
                              /* point of interest: */const owl::vec3f &interest,
                              /* up-vector        : */const owl::vec3f &up,
                              /* fovy, in degrees : */float fovyInDegrees);

    void loadTransferFunction(const std::string &fileName);


  // protected:


    owl::vec2i  fbSize { 0 };
    uint32_t    *fbPointer { nullptr };
    std::string title;
    qtOWL::Camera camera;

    struct {
      float opacityScale { 100.f };
      owl::interval<float> absDomain { 0.f, 1.f };
      owl::interval<float> relDomain { 0.f, 99.f };
      std::vector<owl::vec4f> colorMap;
    } xf;

    owl::interval<float> xfRange();

    std::string outFileName;
    std::string outFileNameFPS;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

