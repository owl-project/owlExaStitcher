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

#if _WIN32 // need to include windows.h before gl.h
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <owl/common/math/box.h>
#include "LightInteractor.h"

using namespace owl;

namespace gl {
  struct mat4
  {
    mat4() = default;

    mat4(float m00, float m10, float m20, float m30,
         float m01, float m11, float m21, float m31,
         float m02, float m12, float m22, float m32,
         float m03, float m13, float m23, float m33)
    {
      m[ 0] = m00; m[ 1] = m10; m[ 2] = m20; m[ 3] = m30;
      m[ 4] = m01; m[ 5] = m11; m[ 6] = m21; m[ 7] = m31;
      m[ 8] = m02; m[ 9] = m12; m[10] = m22; m[11] = m32;
      m[12] = m03; m[13] = m13; m[14] = m23; m[15] = m33;
    }
 
    static mat4 makeIdentity()
    {
      mat4 result;
      for (int i=0; i<16; ++i) {
        result.m[i] = (i%5==0) ? 1.f : 0.f;
      }
      return result;
    }

    static mat4 makeScale(const vec3f s)
    {
      mat4 result = mat4::makeIdentity();
      result(0,0) = s.x;
      result(1,1) = s.y;
      result(2,2) = s.z;
      return result;
    }

    float& operator()(int x, int y) { return m[y*4+x]; }
    const float& operator()(int x, int y) const { return m[y*4+x]; }
  
    float m[16];
  };
  
  std::ostream& operator<<(std::ostream &out, const mat4 &m)
  {
    out << m.m[ 0] << ' ' << m.m[ 4] << ' ' << m.m[ 8] << ' ' << m.m[12] << '\n'
        << m.m[ 1] << ' ' << m.m[ 5] << ' ' << m.m[ 9] << ' ' << m.m[13] << '\n'
        << m.m[ 2] << ' ' << m.m[ 6] << ' ' << m.m[10] << ' ' << m.m[14] << '\n'
        << m.m[ 3] << ' ' << m.m[ 7] << ' ' << m.m[11] << ' ' << m.m[15];
    return out;
  }

  inline mat4 inverse(const mat4& m)
  {
    auto det2 = [](float m00, float m01, float m10, float m11) {
      return m00 * m11 - m10 * m01;
    };
  
    float s0 = det2(m(0,0), m(0,1), m(1,0), m(1,1));
    float s1 = det2(m(0,0), m(0,2), m(1,0), m(1,2));
    float s2 = det2(m(0,0), m(0,3), m(1,0), m(1,3));
    float s3 = det2(m(0,1), m(0,2), m(1,1), m(1,2));
    float s4 = det2(m(0,1), m(0,3), m(1,1), m(1,3));
    float s5 = det2(m(0,2), m(0,3), m(1,2), m(1,3));
    float c5 = det2(m(2,2), m(2,3), m(3,2), m(3,3));
    float c4 = det2(m(2,1), m(2,3), m(3,1), m(3,3));
    float c3 = det2(m(2,1), m(2,2), m(3,1), m(3,2));
    float c2 = det2(m(2,0), m(2,3), m(3,0), m(3,3));
    float c1 = det2(m(2,0), m(2,2), m(3,0), m(3,2));
    float c0 = det2(m(2,0), m(2,1), m(3,0), m(3,1));
  
    float det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0; 
  
    return mat4(
      (+ m(1,1) * c5 - m(1,2) * c4 + m(1,3) * c3) / det,
      (- m(1,0) * c5 + m(1,2) * c2 + m(1,3) * c1) / det,
      (+ m(1,0) * c4 - m(1,1) * c2 + m(1,3) * c0) / det,
      (- m(1,0) * c3 + m(1,1) * c1 + m(1,2) * c0) / det,
      (- m(0,1) * c5 + m(0,2) * c4 - m(0,3) * c3) / det,
      (+ m(0,0) * c5 - m(0,2) * c2 + m(0,3) * c1) / det,
      (- m(0,0) * c4 + m(0,1) * c2 - m(0,3) * c0) / det,
      (+ m(0,0) * c3 - m(0,1) * c1 + m(0,2) * c0) / det,
      (+ m(3,1) * s5 - m(3,2) * s4 + m(3,3) * s3) / det,
      (- m(3,0) * s5 + m(3,2) * s2 - m(3,3) * s1) / det,
      (+ m(3,0) * s4 - m(3,1) * s2 + m(3,3) * s0) / det,
      (- m(3,0) * s3 + m(3,1) * s1 - m(3,2) * s0) / det,
      (- m(2,1) * s5 + m(2,2) * s4 - m(2,3) * s3) / det,
      (+ m(2,0) * s5 - m(2,2) * s2 + m(2,3) * s1) / det,
      (- m(2,0) * s4 + m(2,1) * s2 - m(2,3) * s0) / det,
      (+ m(2,0) * s3 - m(2,1) * s1 + m(2,2) * s0) / det 
    );  
  }

  inline mat4 operator*(const mat4 &a, const mat4 &b)
  {
    return mat4(
      a(0,0) * b(0,0) + a(0,1) * b(1,0) + a(0,2) * b(2,0) + a(0,3) * b(3,0),
      a(1,0) * b(0,0) + a(1,1) * b(1,0) + a(1,2) * b(2,0) + a(1,3) * b(3,0),
      a(2,0) * b(0,0) + a(2,1) * b(1,0) + a(2,2) * b(2,0) + a(2,3) * b(3,0),
      a(3,0) * b(0,0) + a(3,1) * b(1,0) + a(3,2) * b(2,0) + a(3,3) * b(3,0),
      a(0,0) * b(0,1) + a(0,1) * b(1,1) + a(0,2) * b(2,1) + a(0,3) * b(3,1),
      a(1,0) * b(0,1) + a(1,1) * b(1,1) + a(1,2) * b(2,1) + a(1,3) * b(3,1),
      a(2,0) * b(0,1) + a(2,1) * b(1,1) + a(2,2) * b(2,1) + a(2,3) * b(3,1),
      a(3,0) * b(0,1) + a(3,1) * b(1,1) + a(3,2) * b(2,1) + a(3,3) * b(3,1),
      a(0,0) * b(0,2) + a(0,1) * b(1,2) + a(0,2) * b(2,2) + a(0,3) * b(3,2),
      a(1,0) * b(0,2) + a(1,1) * b(1,2) + a(1,2) * b(2,2) + a(1,3) * b(3,2),
      a(2,0) * b(0,2) + a(2,1) * b(1,2) + a(2,2) * b(2,2) + a(2,3) * b(3,2),
      a(3,0) * b(0,2) + a(3,1) * b(1,2) + a(3,2) * b(2,2) + a(3,3) * b(3,2),
      a(0,0) * b(0,3) + a(0,1) * b(1,3) + a(0,2) * b(2,3) + a(0,3) * b(3,3),
      a(1,0) * b(0,3) + a(1,1) * b(1,3) + a(1,2) * b(2,3) + a(1,3) * b(3,3),
      a(2,0) * b(0,3) + a(2,1) * b(1,3) + a(2,2) * b(2,3) + a(2,3) * b(3,3),
      a(3,0) * b(0,3) + a(3,1) * b(1,3) + a(3,2) * b(2,3) + a(3,3) * b(3,3)
    );
  }

  inline owl::vec4f operator*(const mat4 &m, const owl::vec4f &v)
  {
    return vec4f(
      m(0,0) * v.x + m(0,1) * v.y + m(0,2) * v.z + m(0,3) * v.w,
      m(1,0) * v.x + m(1,1) * v.y + m(1,2) * v.z + m(1,3) * v.w,
      m(2,0) * v.x + m(2,1) * v.y + m(2,2) * v.z + m(2,3) * v.w,
      m(3,0) * v.x + m(3,1) * v.y + m(3,2) * v.z + m(3,3) * v.w
    );
  }

  inline void project(owl::vec3f       &win,
                      const owl::vec3f &obj,
                      const gl::mat4   &modelview,
                      const gl::mat4   &projection,
                      const owl::vec4i &viewport)
  {
    const owl::vec4f tmp = projection * modelview * owl::vec4f(obj,1.f);
  
    const owl::vec3f v = owl::vec3f(tmp.x,tmp.y,tmp.z)/tmp.w;
  
    win.x = viewport[0]+viewport[2] * (v.x+1.f) / 2.f;
    win.y = viewport[1]+viewport[3] * (v.y+1.f) / 2.f;
    win.z = (v.z+1.f)/2.f;
  }

  inline void unproject(owl::vec3f       &obj,
                        owl::vec3f       &win,
                        const gl::mat4   &modelview,
                        const gl::mat4   &projection,
                        const owl::vec4i &viewport)
  {
    vec4f u(2.f * (win.x-viewport[0]) / viewport[2]-1.f,
            2.f * (win.y-viewport[1]) / viewport[3]-1.f,
            2.f * win.z-1.f,
            1.f);
  
    const mat4 invPM = inverse(projection * modelview);
  
    owl::vec4f v = invPM * u;
  
    obj = owl::vec3f(v.x,v.y,v.z)/v.w;
  }
} // ::gl


namespace exa {
  void LightInteractor::setWorldScale(const float s)
  {
    scale = s;
  }

  void LightInteractor::update(const float *view, const float *proj, const owl::vec2i fbSize)
  {
    memcpy(this->view,view,sizeof(float)*16);
    memcpy(this->proj,proj,sizeof(float)*16);
    this->fbSize = fbSize;
  }

  void LightInteractor::draw() const
  {
    // store GL state
    glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT);
  
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
  
    float r = scale/10.f;
  
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(pos[0], pos[1], pos[2]);
    GLUquadricObj* quad = gluNewQuadric();
  
    glColor3f(1.0f, 1.0f, 0.0f);
    gluSphere(quad, r, 10.0f, 10.0f);
  
    // axes
    if (hasFocus())
    {
      glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(r, 0.0f, 0.0f);
        glVertex3f(5 * r, 0.0f, 0.0f);
  
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, r, 0.0f);
        glVertex3f(0.0f, 5 * r, 0.0f);
  
        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, r);
        glVertex3f(0.0f, 0.0f, 5 * r);
      glEnd();
  
      GLUquadricObj* quadx = gluNewQuadric();
      glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
      glTranslatef(0.0f, 0.0f, 5 * r);
      glColor3f(1.0f, 0.0f, 0.0f);
      gluCylinder(quadx, r / 2.0f, 0.0f, r * 2.0f, 10.0f, 10.0f);
      glTranslatef(0.0f, 0.0f, -(5 * r));
      glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
  
      GLUquadricObj* quady = gluNewQuadric();
      glRotatef(270.0f, 1.0f, 0.0f, 0.0f);
      glTranslatef(0.0f, 0.0f, 5 * r);
      glColor3f(0.0f, 1.0f, 0.0f);
      gluCylinder(quady, r / 2.0f, 0.0f, r * 2.0f, 10.0f, 10.0f);
      glTranslatef(0.0f, 0.0f, -(5 * r));
      glRotatef(-270.0f, 1.0f, 0.0f, 0.0f);
  
      GLUquadricObj* quadz = gluNewQuadric();
      glTranslatef(0.0f, 0.0f, 5 * r);
      glColor3f(0.0f, 0.0f, 1.0f);
      gluCylinder(quadz, r / 2.0f, 0.0f, r * 2.0f, 10.0f, 10.0f);
      glTranslatef(0.0f, 0.0f, -(5 * r));
    }
  
    glPopMatrix();
  
    glPopAttrib();
  }
  
  void LightInteractor::mouseButtonLeft(const vec2i &where, bool pressed)
  {
    if (!pressed)
      emit lightPosChanged(pos);
  }

  void LightInteractor::mouseDragLeft(const vec2i &where, const vec2i &delta)
  {
    gl::mat4 mv, pr;
    memcpy(mv.m,this->view,sizeof(this->view));
    memcpy(pr.m,this->proj,sizeof(this->proj));
    vec4i vp(0,0,fbSize.x,fbSize.y);

    vec3f obj;
    gl::project(obj,pos,mv,pr,vp);
    vec3f win(where.x,vp[3]-where.y-1,obj.z);
    gl::unproject(pos,win,mv,pr,vp);

    emit lightPosChanged(pos);
  }

  void LightInteractor::setPos(const owl::vec3f p)
  {
    pos = p;
  }


} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

