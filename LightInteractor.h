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

#include <QObject>
#include <owl/common/math/vec.h>

namespace exa {
  
  class LightInteractor : public QObject
  {
    Q_OBJECT
  public:

    void setWorldScale(const float s);

    void update(const float *view, const float *proj, const owl::vec2i fbSize);

    void draw() const;

    void mouseButtonLeft(const owl::vec2i &where, bool pressed);
    void mouseDragLeft(const owl::vec2i &where, const owl::vec2i &delta);

    void setPos(const owl::vec3f p);

    void toggleActive() { active_ = !active_; }
    bool active() const { return active_; }
    bool hasFocus() const { return true; }

  signals:
    void lightPosChanged(owl::vec3f);
  private:
    owl::vec3f pos{ 0.f };
    bool active_{false};
    float scale = 3.f;
    float view[16];
    float proj[16];
    owl::vec2i fbSize;
  };

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

