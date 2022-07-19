// ======================================================================== //
// Copyright 2020 Ingo Wald                                                 //
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

namespace exa {
  
#ifdef __CUDA_ARCH__
  struct Plane {
    inline __device__ float eval(const vec3f v) const
    { return dot(v,N)-d; }
    inline __device__ float eval(const vec4f v) const
    { return dot((const vec3f&)v,N)-d; }
    
    vec3f N;
    float d;
  };
  inline __device__ Plane makePlane(const vec3f a,
                                    const vec3f b,
                                    const vec3f c)
  {
    vec3f N = cross(b-a,c-a);
    return { N,dot(a,N) };
  }
  inline __device__ Plane makePlane(const vec4f a,
                                    const vec4f b,
                                    const vec4f c)
  {
    return makePlane((const vec3f&)a,
                     (const vec3f&)b,
                     (const vec3f&)c);
  }
  inline __device__ Plane makePlane(const float4 a,
                                    const float4 b,
                                    const float4 c)
  {
    return makePlane((const vec3f&)a,
                     (const vec3f&)b,
                     (const vec3f&)c);
  }


  inline __both__
  bool intersectTet(float &value,
                    const vec3f P,
                    const float4 a,
                    const float4 b,
                    const float4 c,
                    const float4 d)
  {
    // if (dbg()) {
    //   printf("  A> %f %f %f : %f\n",
    //          a.x,a.y,a.z,a.w);
    //   printf("  B> %f %f %f : %f\n",
    //          b.x,b.y,b.z,b.w);
    //   printf("  C> %f %f %f : %f\n",
    //          c.x,c.y,c.z,c.w);
    //   printf("  D> %f %f %f : %f\n",
    //          d.x,d.y,d.z,d.w);
    // }
    vec3f va = vec3f(a)-P;
    vec3f vb = vec3f(b)-P;
    vec3f vc = vec3f(c)-P;
    vec3f vd = vec3f(d)-P;

    Plane pa = makePlane(vb,vd,vc);
    Plane pb = makePlane(va,vc,vd);
    Plane pc = makePlane(va,vd,vb);
    Plane pd = makePlane(va,vb,vc);

    float fa = pa.eval(vec3f(0.f))/pa.eval(va);
    if (fa < 0.f || fa > 1.f) return false;
    
    float fb = pb.eval(vec3f(0.f))/pb.eval(vb);
    if (fb < 0.f || fa > 1.f) return false;
    
    float fc = pc.eval(vec3f(0.f))/pc.eval(vc);
    if (fc < 0.f || fa > 1.f) return false;
    
    float fd = pd.eval(vec3f(0.f))/pd.eval(vd);
    if (fd < 0.f || fa > 1.f) return false;

    value = fa*a.w + fb*b.w + fc*c.w + fd*d.w;
    return true;
  }
  
  inline __both__
  bool intersectPair(float &value,
                     const vec3f &P,
                     const float4 a,
                     const float4 b,
                     const float4 c,
                     const float4 d0,
                     const float4 d1)
  {
    if (intersectTet(value,P,a,b,c,d0)) return true;
    if (intersectTet(value,P,a,c,b,d1)) return true;

    return false;
  }

  inline  __both__
  bool intersectPyr(float &value,
                    const vec3f &P,
                    const float4 _v0,
                    const float4 _v1,
                    const float4 _v2,
                    const float4 _v3,
                    const float4 _v4
                    )
  {
    const float f0 = _v0.w;
    const float f1 = _v1.w;
    const float f2 = _v2.w;
    const float f3 = _v3.w;
    const float f4 = _v4.w;

    const vec3f p0 = vec3f(_v0);
    const vec3f p1 = vec3f(_v1);
    const vec3f p2 = vec3f(_v2);
    const vec3f p3 = vec3f(_v3);
    const vec3f p4 = vec3f(_v4);

    Plane base = makePlane(p0,p1,p2);
    float w = base.eval(P)/base.eval(p4);
    if (w > 1.f && w < 0.f) return false;

    const float u0 = makePlane(p0,p4,p1).eval(P);
    if (u0 < 0.f) return false;
    const float u1 = makePlane(p2,p4,p3).eval(P);
    if (u1 < 0.f) return false;
    const float u = u0 / (u0+u1+1e-10f);
    
    const float v0 = makePlane(p0,p3,p4).eval(P);
    if (v0 < 0.f) return false;
    const float v1 = makePlane(p1,p4,p2).eval(P);
    if (v1 < 0.f) return false;
    const float v = v0 / (v0+v1+1e-10f);
    
    // prd.primID = primID;
    value = w*f4 + (1.f-w)*((1.f-u)*(1.f-v)*f0+
                            (1.f-u)*(    v)*f1+
                            (    u)*(1.f-v)*f3+
                            (    u)*(    v)*f2);
    return true;
  }

   inline  __both__
  bool intersectWedge(float &value,
                      const vec3f &P,
                      const float4 _v0,
                      const float4 _v1,
                      const float4 _v2,
                      const float4 _v3,
                      const float4 _v4,
                      const float4 _v5)
  {
    const float f0 = _v0.w;
    const float f1 = _v1.w;
    const float f2 = _v2.w;
    const float f3 = _v3.w;
    const float f4 = _v4.w;
    const float f5 = _v5.w;
    const vec3f p0 = vec3f(_v0);
    const vec3f p1 = vec3f(_v1);
    const vec3f p2 = vec3f(_v2);
    const vec3f p3 = vec3f(_v3);
    const vec3f p4 = vec3f(_v4);
    const vec3f p5 = vec3f(_v5);

    Plane base = makePlane(p0,p1,p3);
    const float w0 = base.eval(P);
    if (w0 < 0.f) return false;
    
    Plane top;
    top.N = cross(cross(base.N,p5-p2),p5-p2);
    top.d = dot(top.N,p2);
    const float w1 = top.eval(P);
    if (w1 < 0.f) return false;
    const float w = w0/(w0+w1+1e-10f);

    Plane front = makePlane(p0,p2,p1);
    Plane back  = makePlane(p3,p4,p5);
    const float u0 = front.eval(P);
    if (u0 < 0.f) return false;
    const float u1 = back.eval(P);
    if (u1 < 0.f) return false;
    const float u = u0/(u0+u1+1e-10f);

    Plane left = makePlane(p0,p3,p2);
    Plane right  = makePlane(p1,p2,p4);
    const float v0 = left.eval(P);
    if (v0 < 0.f) return false;
    const float v1 = right.eval(P);
    if (v1 < 0.f) return false;
    const float v = v0/(v0+v1+1e-10f);

    const float fbase
      = (1.f-u)*(1.f-v)*f0
      + (1.f-u)*(    v)*f1
      + (    u)*(1.f-v)*f3
      + (    u)*(    v)*f4;
    const float ftop = (1.f-u)*f2 + u*f5;
    // prd.value = (1.f-w)*fbase + w*ftop;
    // prd.primID = primID;
    value = (1.f-w)*fbase + w*ftop;
    return true;
  }

  inline __device__
  bool intersectHex(float &value,
                    const vec3f &P,
                    const float4 v0,
                    const float4 v1,
                    const float4 v2,
                    const float4 v3,
                    const float4 v4,
                    const float4 v5,
                    const float4 v6,
                    const float4 v7)
  {
    float f0 = v0.w;
    float f1 = v1.w;
    float f2 = v2.w;
    float f3 = v3.w;
    float f4 = v4.w;
    float f5 = v5.w;
    float f6 = v6.w;
    float f7 = v7.w;

    const Plane frt = makePlane(v0,v4,v1);
    const Plane bck = makePlane(v3,v2,v7);
    const Plane lft = makePlane(v0,v3,v4);
    const Plane rgt = makePlane(v1,v5,v2);
    const Plane top = makePlane(v4,v7,v5);
    const Plane btm = makePlane(v0,v1,v3);

    const float t_frt = frt.eval(P); if (t_frt < 0.f) return false;
    const float t_bck = bck.eval(P); if (t_bck < 0.f) return false;
    const float t_lft = lft.eval(P); if (t_lft < 0.f) return false;
    const float t_rgt = rgt.eval(P); if (t_rgt < 0.f) return false;
    const float t_top = top.eval(P); if (t_top < 0.f) return false;
    const float t_btm = btm.eval(P); if (t_btm < 0.f) return false;

    const float f_x = t_lft/(t_lft+t_rgt);
    const float f_y = t_frt/(t_frt+t_bck);
    const float f_z = t_btm/(t_btm+t_top);
    
    value// = 1.f/8.f *(f0+f1+f2+f3+f4+f5+f6+f7);
      =
      + (1.f-f_z)*(1.f-f_y)*(1.f-f_x)*f0
      + (1.f-f_z)*(1.f-f_y)*(    f_x)*f1
      + (1.f-f_z)*(    f_y)*(1.f-f_x)*f3
      + (1.f-f_z)*(    f_y)*(    f_x)*f2
      + (    f_z)*(1.f-f_y)*(1.f-f_x)*f4
      + (    f_z)*(1.f-f_y)*(    f_x)*f5
      + (    f_z)*(    f_y)*(1.f-f_x)*f7
      + (    f_z)*(    f_y)*(    f_x)*f6
      ;
    return true;
  }






  


  






  inline __both__
  bool intersectTet(float &value,
                    const vec3f &P,
                    const vec4f *vertices,
                    const ushort *indices)
  {
    float4 a = vertices[indices[0]];
    float4 b = vertices[indices[1]];
    float4 c = vertices[indices[2]];
    float4 d = vertices[indices[3]];
    return intersectTet(value,P,a,b,c,d);
  }

  inline __both__
  bool intersectPair(float &value,
                     const vec3f &P,
                     const vec4f *vertices,
                     const ushort *indices)
  {
    float4 a = vertices[indices[0]];
    float4 b = vertices[indices[1]];
    float4 c = vertices[indices[2]];
    float4 d0 = vertices[indices[3]];
    float4 d1 = vertices[indices[4]];

    return intersectPair(value,P,a,b,c,d0,d1);
  }
  

  inline  __both__

  bool intersectPyr(float &value,
                    const vec3f &P,
                    const vec4f *vertices,
                    const ushort *indices)
  {
    const float4 _v0 = vertices[indices[0]];
    const float4 _v1 = vertices[indices[1]];
    const float4 _v2 = vertices[indices[2]];
    const float4 _v3 = vertices[indices[3]];
    const float4 _v4 = vertices[indices[4]];
    return intersectPyr(value,P,_v0,_v1,_v2,_v3,_v4);
  }
  
  inline  __both__
  bool intersectWedge(float &value,
                      const vec3f &P,
                      const vec4f *vertices,
                      const ushort *indices)
  {
    const float4 _v0 = vertices[indices[0]];
    const float4 _v1 = vertices[indices[1]];
    const float4 _v2 = vertices[indices[2]];
    const float4 _v3 = vertices[indices[3]];
    const float4 _v4 = vertices[indices[4]];
    const float4 _v5 = vertices[indices[5]];
    return intersectWedge(value,P,_v0,_v1,_v2,_v3,_v4,_v5);
  }
  
  inline  __both__
  bool intersectHex(float &value,
                    const vec3f &P,
                    const vec4f *vertices,
                    const ushort *indices)
  {
    float4 v0 = vertices[indices[0]];
    float4 v1 = vertices[indices[1]];
    float4 v2 = vertices[indices[2]];
    float4 v3 = vertices[indices[3]];
    float4 v4 = vertices[indices[4]];
    float4 v5 = vertices[indices[5]];
    float4 v6 = vertices[indices[6]];
    float4 v7 = vertices[indices[7]];
    return intersectHex(value,P,
                        v0,v1,v2,v3,v4,v5,v6,v7);
  }

  
#endif  
  
}
