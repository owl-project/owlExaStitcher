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

#include <iomanip>
#include <fstream>
#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include "qtOWL/OWLViewer.h"
#include "qtOWL/XFEditor.h"
#include "LightInteractor.h"
#include "OWLRenderer.h"
#ifdef HEADLESS
#include "headless.h"
#endif

#define VOLKD_HAVE_OWN_MATH
#define VOLKD_PARALLEL_SWEEP
#include "YueKDTree.h"
#include "YueVolume.h"


#define DUMP_FRAMES 0

static int g_clipPlaneSelected = 0;

namespace exa {
  using qtOWL::Camera;
  using qtOWL::SimpleCamera;

  struct {
    std::string scalarFileName = "";
    std::string gridsFileName = "";
    std::string amrCellFileName = "";
    std::string exaBrickFileName = "";
    std::string bigMeshFileName = "";
    std::string quickClustersFileName = "";
    std::string kdtreeFileName = "";
    std::string majorantsFileName = "";
    std::string meshFileName = "";
    std::string xfFileName = "";
    std::string outFileName = "Witcher3.png";
    std::string fpsFileName = "fps.log";
    range1f valueRange = {1e30f,-1e30f};
    box2f subImage = {{1e30f,1e30f},{-1e30f,-1e30f}};
    struct {
      vec3f vp = vec3f(0.f);
      vec3f vu = vec3f(0.f);
      vec3f vi = vec3f(0.f);
      float fovy = 70;
    } camera;
    struct {
      int enabled = 0;
      int show = 1;
      vec3f N{0,0,1};
      float d{1000.f};
    } clipPlanes[CLIP_PLANES_MAX];
    struct {
      box3f remap_from { vec3f(0.f), vec3f(1.f) };
      box3f remap_to   { vec3f(0.f), vec3f(1.f) };
    } xform;
    int rendererType = 0;
    int shadeMode = 0;
    struct {
      vec3f pos{0,0,0};
      float intensity{ 0.f };
      bool on = false;
    } lights[1];
    vec3i numMCs{128,128,128};
    vec2i windowSize  = vec2i(1024,1024);
    float dt = .5f;
    int spp = 1;
    int heatMapEnabled = 0;
    float heatMapScale = 1.f;
  } cmdline;
  
  void usage(const std::string &err)
  {
    if (err != "")
      std::cout << OWL_TERMINAL_RED << "\nFatal error: " << err
                << OWL_TERMINAL_DEFAULT << std::endl << std::endl;

    std::cout << "Usage: ./owlDVR volumeFile.raw -dims x y z -f|--format float|byte" << std::endl;
    std::cout << std::endl;
    exit(1);
  }

  struct Mat4 {
    float m[16];
  };

  inline Mat4 lookAt(const vec3f eye, const vec3f center, const vec3f up)
  {
    const vec3f f = normalize(eye-center);
    const vec3f s = normalize(cross(up,f));
    const vec3f u = cross(f,s);
    return {
      s.x, u.x, f.x, 0.f,
      s.y, u.y, f.y, 0.f,
      s.z, u.z, f.z, 0.f,
      -dot(eye,s), -dot(eye,u), -dot(eye,f), 1.f
    };
  }

  inline Mat4 perspective(float fovy, float aspect, float znear=.001f, float zfar=1000.f)
  {
    #define cot(x) cos(x)/sin(x)
    float f = cot(fovy*.5f);
    #undef cot

    return {
      f/aspect, 0.f, 0.f, 0.f,
      0.f, f, 0.f, 0.f,
      0.f, 0.f, (zfar+znear)/(znear-zfar), -1.f,
      0.f, 0.f, (2.f*zfar*znear)/(znear-zfar), 0.f
    };
  }

  inline float intersectLinePlane(const vec3f &p1,
                                  const vec3f &p2,
                                  const vec3f &normal,
                                  float offset)
  {
    float s = dot(normal, normalize(p2-p1));

    if (s == 0.f)
      return -1.f;

    float t = (offset - dot(normal, p1)) / s;

    if (t < 0.f || t > length(p2-p1))
      return -1.f;

    return t;
  }

  inline void intersectBoxPlane(const box3f &box,
                                const vec3f &normal,
                                float offset,
                                vec3f *pts,
                                int& isectCnt)
  {
    const int key[4][3] = { {0,0,0}, {1,0,1}, {1,1,0}, {0,1,1} };
    vec3f corners[2] = { box.lower, box.upper };
    isectCnt = 0;

    for (int i=0; i<4 && isectCnt<6; ++i) {
      for (int j=0; j<3 && isectCnt<6; ++j) {
        // Edge to intersect with
        vec3f p1((j==0) ? (corners[1-key[i][0]][0]) : corners[key[i][0]][0],
          (j==1) ? (corners[1-key[i][1]][1]) : corners[key[i][1]][1],
          (j==2) ? (corners[1-key[i][2]][2]) : corners[key[i][2]][2]);
        vec3f p2(corners[key[i][0]][0], corners[key[i][1]][1], corners[key[i][2]][2]);

        // Intersect edge with plane
        float t = intersectLinePlane(p1,p2,normal,offset);

        if (t >= 0.f) {
          pts[isectCnt++] = p1 + normalize(p2-p1) * t;
        }
      }
    }
  }

#ifdef HEADLESS
  struct Viewer : public Headless {
    typedef Headless inherited;
#else
  struct Viewer : public qtOWL::OWLViewer {
    typedef qtOWL::OWLViewer inherited;
#endif

    Q_OBJECT

  public:
    Viewer(OWLRenderer *renderer)
      : inherited("exastitch", cmdline.windowSize)
      , renderer(renderer)
    {
    }

    /*! this function gets called whenever the viewer widget changes
      camera settings */
    void cameraChanged() override;
    void resize(const vec2i &newSize) override;
    /*! gets called whenever the viewer needs us to re-render out widget */
    void render() override;
      
    /*! draw framebuffer using OpenGL */
    void draw();

    /*! this gets called when the user presses a key on the keyboard ... */
    void key(char key, const vec2i &where) override
    {
      inherited::key(key,where);
      renderer->resetAccum();
      switch (key) {
      case '!':
        std::cout << "saving screenshot to '" << cmdline.outFileName << ".png'\n";
        screenShot(cmdline.outFileName+".png");
        break;
      case 'H':
        renderer->heatMapEnabled = !renderer->heatMapEnabled;
        break;
      case 'l':
        emit lightEdittingToggled();
        break;
      case '<':
        renderer->heatMapScale /= 1.5f;
        break;
      case '>':
        renderer->heatMapScale *= 1.5f;
        break;
      case ')':
        renderer->spp++;
        PRINT(renderer->spp);
        break;
      case '(':
        renderer->spp = max(1,renderer->spp-1);
        PRINT(renderer->spp);
        break;
      case 'T':
        if (xfEditor) xfEditor->saveTo("owlDVR.xf");
        break;
      case 'y': {
        if (auto model = renderer->model->as<ExaBrickModel>()) {
          YueVolume vol;
          vol.cellBounds = box3i(vec3i(model->cellBounds.lower),
                                 vec3i(model->cellBounds.upper));
          vol.valueRange = model->valueRange;
          ExaBrickSamplerCPU::SP sampler = std::make_shared<ExaBrickSamplerCPU>();
          sampler->build(model);
          vol.sampler = sampler;

          std::vector<float> rgbaCM;
          std::vector<volkd::KDTreeNode> nodes;

          std::string baseFileName = cmdline.scalarFileName;
          std::string cmFileName = baseFileName+"_MajorantColorMap.bin";
          std::string kdTreeFileName = baseFileName+"_MajorantKDTree.bin";
          std::string domainsFileName = baseFileName+"_MajorantDomains.bin";

          std::ifstream cmFile(cmFileName);
          uint64_t cmSize = 0;
          cmFile.read((char *)&cmSize,sizeof(cmSize));
          rgbaCM.resize(cmSize);
          cmFile.read((char *)rgbaCM.data(),rgbaCM.size()*sizeof(rgbaCM[0]));

          std::ifstream kdTreeFile(kdTreeFileName);
          uint64_t numNodes = 0;
          kdTreeFile.read((char *)&numNodes,sizeof(numNodes));
          nodes.resize(numNodes);
          kdTreeFile.read((char *)nodes.data(),nodes.size()*sizeof(nodes[0]));

          std::vector<std::pair<box3i,float>> domains;
          for (size_t i=0; i<nodes.size(); ++i) {
            if (nodes[i].childIDs[0] < 0 && nodes[i].childIDs[1] < 0) {
              float majorant = vol.boundsFindMajorant(nodes[i].domain,&rgbaCM);
              std::cout << "Domain " << domains.size() << ": "
                        << nodes[i].domain << ", majorant: "
                        << majorant << '\n';
              domains.push_back({nodes[i].domain,majorant});
            }
          }

          std::ofstream domainsFile(domainsFileName);
          uint64_t numDomains = domains.size();
          domainsFile.write((char *)&numDomains,sizeof(numDomains));
          domainsFile.write((char *)domains.data(),domains.size()*sizeof(domains[0]));
        }
        break;
      }
      case 'Y': {
        if (auto model = renderer->model->as<ExaBrickModel>()) {
          YueVolume vol;
          vol.cellBounds = box3i(vec3i(model->cellBounds.lower),
                                 vec3i(model->cellBounds.upper));
          vol.valueRange = model->valueRange;
          ExaBrickSamplerCPU::SP sampler = std::make_shared<ExaBrickSamplerCPU>();
          sampler->build(model);
          vol.sampler = sampler;
          std::vector<float> rgbaCM(xfEditor->getColorMap().size()*4);
          memcpy(rgbaCM.data(),xfEditor->getColorMap().data(),
                 rgbaCM.size()*sizeof(rgbaCM[0]));
          volkd::KDTree kdtree(vol,&rgbaCM);

          std::vector<std::pair<box3i,float>> domains;
          for (size_t i=0; i<kdtree.nodes.size(); ++i) {
            if (kdtree.nodes[i].childIDs[0] < 0 && kdtree.nodes[i].childIDs[1] < 0) {
              float majorant = vol.boundsFindMajorant(kdtree.nodes[i].domain,&rgbaCM);
              std::cout << "Domain " << domains.size() << ": "
                        << kdtree.nodes[i].domain << ", majorant: "
                        << majorant << '\n';
              domains.push_back({kdtree.nodes[i].domain,majorant});
            }
          }

          std::string baseFileName = cmdline.scalarFileName;
          std::string cmFileName = baseFileName+"_MajorantColorMap.bin";
          std::string kdTreeFileName = baseFileName+"_MajorantKDTree.bin";
          std::string domainsFileName = baseFileName+"_MajorantDomains.bin";

          std::ofstream cmFile(cmFileName);
          uint64_t cmSize = rgbaCM.size();
          cmFile.write((char *)&cmSize,sizeof(cmSize));
          cmFile.write((char *)rgbaCM.data(),rgbaCM.size()*sizeof(rgbaCM[0]));

          std::ofstream kdTreeFile(kdTreeFileName);
          uint64_t numNodes = kdtree.nodes.size();
          kdTreeFile.write((char *)&numNodes,sizeof(numNodes));
          kdTreeFile.write((char *)kdtree.nodes.data(),
                           kdtree.nodes.size()*sizeof(kdtree.nodes[0]));

          std::ofstream domainsFile(domainsFileName);
          uint64_t numDomains = domains.size();
          domainsFile.write((char *)&numDomains,sizeof(numDomains));
          domainsFile.write((char *)domains.data(),domains.size()*sizeof(domains[0]));
        }
        break;
      }
      case 'P':
        if (g_clipPlaneSelected >= 0 && cmdline.clipPlanes[g_clipPlaneSelected].enabled) {
          const vec3f N = cmdline.clipPlanes[g_clipPlaneSelected].N;
          const float d = cmdline.clipPlanes[g_clipPlaneSelected].d;
          std::cout << "(suggested cmdline format, for apps that support this:) "
                    << std::endl
                    << " --clip-plane"
                    << " " << N.x << " " << N.y << " " << N.z << " " << d
                    << std::endl;
        }
        break;
      }
    }

    void mouseButtonLeft(const vec2i &where, bool pressed)
    {
      if (subImageSelecting) {
        if (!pressed && !lastSubImageWin.empty()) {
          const box2f subImageUV = lastSubImageWin;
          pushSubImage(subImageUV);
          box2f combinedUV = computeSubImageUV();
          renderer->setSubImage(combinedUV,true);
          emit subImageChanged(combinedUV);
          renderer->setSubImageSelection({},false); // deactivate selection
          lastSubImageWin = {};
        }
        downPos = where;
      } else if (lightInteractor.active()) {
        float distToCenter = length(renderer->modelBounds.center() - camera.getFrom());
        float zNear = distToCenter - length(renderer->modelBounds.span())/2.f;
        float zFar  = distToCenter + length(renderer->modelBounds.span())/2.f;
        Mat4 view = lookAt(camera.getFrom(),
                           camera.getAt(),
                           camera.getUp());
        Mat4 proj = perspective(camera.getFovyInDegrees()*M_PI/180.f,
                                float(fbSize.x)/fbSize.y,
                                zNear,zFar);

        
        lightInteractor.update(view.m,proj.m,fbSize);
        lightInteractor.mouseButtonLeft(where,pressed);
      } else {
        inherited::mouseButtonLeft(where,pressed);
      }
    }

    void mouseDragLeft(const vec2i &where, const vec2i &delta)
    {
      if (subImageSelecting) {
        const vec2i size = getWindowSize();
        auto flip = [size](const vec2i P) { return vec2i{P.x,size.y-P.y-1}; };
        vec2i p1 = flip(downPos), p2 = flip(where);
        if (subImageSelectionContraint != SubImageSelectionContraint::Free) {
          vec2i boxSize(std::abs(p2.x-p1.x),std::abs(p2.y-p1.y));
          if (subImageSelectionContraint == SubImageSelectionContraint::KeepAspect) {
            float aspect = getWindowSize().x/(float)getWindowSize().y;
            boxSize.y = boxSize.x/aspect;
          } else if (subImageSelectionContraint == SubImageSelectionContraint::Square) {
            boxSize.x = std::min(boxSize.x,boxSize.y);
            boxSize.y = std::min(boxSize.x,boxSize.y);
          }
          if (p2.x < p1.x) p2.x = p1.x-boxSize.x;
          else if (p2.x > p1.x) p2.x = p1.x+boxSize.x;
          if (p2.y < p1.y) p2.y = p1.y-boxSize.y;
          else if (p2.y > p1.y) p2.y = p1.y+boxSize.y;
        }
        box2i subImageWin = {
          clamp(vec2i(min(p1,p2)),vec2i(0),size),
          clamp(vec2i(max(p1,p2)),vec2i(0),size)
        };
        const box2f subImageUV(vec2f(subImageWin.lower)/vec2f(size),
                               vec2f(subImageWin.upper)/vec2f(size));
        renderer->setSubImageSelection(subImageUV,true);
        lastSubImageWin = subImageUV;
        emit subImageSelectionChanged(subImageUV);
      } else if (lightInteractor.active()) {
        lightInteractor.mouseDragLeft(where,delta);
      } else {
        inherited::mouseDragLeft(where,delta);
      }
    }

  signals:
    void subImageChanged(box2f subImageUV);
    void subImageSelectionChanged(box2f subImageUV);
    void lightEdittingToggled();

  public slots:
    void colorMapChanged(qtOWL::XFEditor *xf);
    void rangeChanged(range1f r);
    void opacityScaleChanged(double scale);
    void lightPosChanged(owl::vec3f pos);
    void scheduleScreenShot(double seconds);
    void scheduleScreenShot(int frames);

  public:

    OWLRenderer *const renderer;
    qtOWL::XFEditor *xfEditor = nullptr;

    vec2i downPos { 0, 0 };
    box2f lastSubImageWin;
    bool subImageSelecting = false;
    std::vector<box2f> subImageUndoStack;
    int subImageUndoStackTop = 0;
    LightInteractor lightInteractor;
    double screenShotAfterSec = -1.0;
    double seconds = 0.0;
    int screenShotAfterFrames = -1;
    int frames = 0;

    enum class SubImageSelectionContraint {
      KeepAspect,Square,Free,
    };
    SubImageSelectionContraint subImageSelectionContraint = SubImageSelectionContraint::KeepAspect;

    void pushSubImage(const box2f si)
    {
      // Push, but if the stack and stack's top are unconsistent,
      // make sure we start with a clean slate
      if (subImageUndoStackTop < subImageUndoStack.size())
        subImageUndoStack.resize(subImageUndoStackTop);

      subImageUndoStack.push_back(si);
      subImageUndoStackTop++;
    }

    box2f computeSubImageUV() const
    {
      box2f combinedUV{{0.f,0.f},{1.f,1.f}};
      if (!cmdline.subImage.empty())
        combinedUV = cmdline.subImage;

      for (size_t i=0; i<subImageUndoStackTop; ++i) {
        const box2f &si = subImageUndoStack[i];
        vec2f currSize = combinedUV.size();
        combinedUV.lower += si.lower*currSize;
        combinedUV.upper = combinedUV.lower + (si.upper-si.lower)*currSize;
      }
      return combinedUV;
    }
  };

  void Viewer::resize(const vec2i &newSize) 
  {
    // ... tell parent to resize (also resizes the pbo in the wingdow)
    inherited::resize(newSize);
    renderer->resize(newSize);
    renderer->resetAccum();
  }
    
  /*! this function gets called whenever the viewer widget changes
    camera settings */
  void Viewer::cameraChanged() 
  {
    inherited::cameraChanged();
    const SimpleCamera &camera = inherited::getCamera();
    
    const vec3f screen_du = camera.screen.horizontal / float(inherited::getWindowSize().x);
    const vec3f screen_dv = camera.screen.vertical   / float(inherited::getWindowSize().y);
    const vec3f screen_00 = camera.screen.lower_left;
    renderer->setCamera(camera.lens.center,screen_00,screen_du,screen_dv);
    renderer->resetAccum();
  }
    

  /*! gets called whenever the viewer needs us to re-render out widget */
  void Viewer::render() 
  {
    if (screenShotAfterFrames >= 1 && frames == 0 ||
        screenShotAfterSec >= 0.0 && seconds == 0.0)
    {
      renderer->resetAccum();
    }

    static double t_last = getCurrentTime();
    static double t_first = t_last;

    renderer->render(inherited::fbPointer);
      
    double t_now = getCurrentTime();
    static double avg_t = t_now-t_last;
    // if (t_last >= 0)
    avg_t = 0.8*avg_t + 0.2*(t_now-t_last);

    char title[1000];
    sprintf(title,"%.2f FPS",(1.f/avg_t));
    inherited::setTitle(title);
    // setWindowTitle(title);
    // glfwSetWindowTitle(this->handle,title);

#ifndef HEADLESS
    if (screenShotAfterFrames >= 0) {
      this->frames++;
      std::cout << frames << '\r';
      std::cout << std::flush;

      if (this->frames >= screenShotAfterFrames) {
        screenShot(cmdline.outFileName+".png");
        screenShotAfterFrames = -1;
        this->frames = 0;
      }
    }

    if (screenShotAfterSec >= 0) {
      this->seconds +=  t_now-t_last;
      std::cout << seconds << '\r';
      std::cout << std::flush;

      if (this->seconds >= screenShotAfterSec) {
        screenShot(cmdline.outFileName+".png");
        screenShotAfterSec = -1.0;
        this->seconds = 0.0;
      }
    }
#endif

    t_last = t_now;

#if DUMP_FRAMES
    // just dump the 10th frame, then hard-exit
    static int g_frameID = 0;
    if (g_frameID % 16 == 0) {
      std::cout << "g_frameID: " << g_frameID << std::endl;
    }

    if (g_frameID++ >= 1024) {
      screenShot(cmdline.outFileName+".png");
      exit(0);
    }
#endif
  }

  /*! draw framebuffer using OpenGL */
  void Viewer::draw()
  {
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    inherited::draw();

    const Camera &camera = inherited::getCamera();

    float distToCenter = length(renderer->modelBounds.center() - camera.getFrom());
    float zNear = distToCenter - length(renderer->modelBounds.span())/2.f;
    float zFar  = distToCenter + length(renderer->modelBounds.span())/2.f;
    Mat4 view = lookAt(camera.getFrom(),
                       camera.getAt(),
                       camera.getUp());
    Mat4 proj = perspective(camera.getFovyInDegrees()*M_PI/180.f,
                            float(fbSize.x)/fbSize.y,
                            zNear,zFar);

    
    glViewport(0, 0, fbSize.x, fbSize.y);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view.m);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.m);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);

    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      if (cmdline.clipPlanes[i].enabled && cmdline.clipPlanes[i].show) {
        vec3f pts[6];
        int isectCnt;
        intersectBoxPlane(renderer->modelBounds,
                          cmdline.clipPlanes[i].N,
                          cmdline.clipPlanes[i].d,
                          pts,isectCnt);

        // Cyclical selection-sort
        for (int ii=0; ii<isectCnt-1; ++ii)
        {
          int minIdx = ii;  
          for (int jj=ii+1; jj<isectCnt; ++jj) {
            vec3f p1 = pts[jj];
            vec3f p2 = pts[minIdx];
            vec3f v = cross(p1-pts[0],p2-pts[0]);
            if (dot(v,cmdline.clipPlanes[i].N) < 0.f)
              minIdx = jj;
          }

          // swap
          vec3f tmp = pts[ii];
          pts[ii] = pts[minIdx];
          pts[minIdx] = tmp;
        }

        glLineWidth(4);
        glColor3f(1.f,1.f,1.f);
        glBegin(GL_LINE_LOOP);
        for (int j=0; j<isectCnt; ++j) {
          glVertex3f(pts[j].x,pts[j].y,pts[j].z);
        }
        glEnd();
      }
    }

    glPopAttrib();

    if (lightInteractor.active())
      lightInteractor.draw();
  }

  void Viewer::colorMapChanged(qtOWL::XFEditor *xfEditor)
  {
    renderer->setColorMap(xfEditor->getColorMap());
    renderer->resetAccum();
  }

  // re-map [0,1] range from gui to actual value range
  void Viewer::rangeChanged(range1f r)
  {
    float lo = min(r.lower,r.upper);
    float hi = max(r.lower,r.upper);
    renderer->setRange({lo,hi});
    renderer->setRelDomain(xfEditor->getRelDomain());
    renderer->resetAccum();
  }

  void Viewer::opacityScaleChanged(double scale)
  {
    renderer->setOpacityScale((float)scale);
    renderer->resetAccum();
  }

  void Viewer::lightPosChanged(owl::vec3f pos)
  {
    renderer->setLightSource(0,pos,cmdline.lights[0].intensity,cmdline.lights[0].on);
    std::ios_base::fmtflags f(std::cout.flags());
    std::cout << std::fixed;
    std::cout << "Cmdline: \"--light "
              << pos.x << ' ' << pos.y << ' ' << pos.z << ' '
              << cmdline.lights[0].intensity
              << "\"\n";
    std::cout.flags(f);
  }

  void Viewer::scheduleScreenShot(double seconds)
  {
    screenShotAfterSec = seconds;
    this->seconds = 0.0;
  }

  void Viewer::scheduleScreenShot(int frames)
  {
    screenShotAfterFrames = frames;
    this->frames = 0;
  }


  extern "C" int main(int argc, char** argv)
  {
    std::string inFileName = "";

    for (int i=1;i<argc;i++) {
      const std::string arg = argv[i];
      if (arg[0] != '-') {
        inFileName = arg;
      }
      else if (arg == "-scalar" || arg == "-scalars") {
        cmdline.scalarFileName = argv[++i];
      }
      else if (arg == "-grids" || arg == "-gridlets") {
        cmdline.gridsFileName = argv[++i];
      }
      else if (arg == "-cells") {
        cmdline.amrCellFileName = argv[++i];
      }
      else if (arg == "-bricks") {
        cmdline.exaBrickFileName = argv[++i];
      }
      else if (arg == "-bm") {
        cmdline.bigMeshFileName = argv[++i];
      }
      else if (arg == "-qc") {
        cmdline.quickClustersFileName = argv[++i];
      }
      else if (arg == "-kdtree") {
        cmdline.kdtreeFileName = argv[++i];
      }
      else if (arg == "-majorants") {
        cmdline.majorantsFileName = argv[++i];
      }
      else if (arg == "-mesh") {
        cmdline.meshFileName = argv[++i];
      }
      else if (arg == "-xf") {
        cmdline.xfFileName = argv[++i];
      }
      else if (arg == "--range") {
        cmdline.valueRange.lower = std::atof(argv[++i]);
        cmdline.valueRange.upper = std::atof(argv[++i]);
      }
      else if (arg == "--subimg") {
        cmdline.subImage.lower.x = std::atof(argv[++i]);
        cmdline.subImage.lower.y = std::atof(argv[++i]);
        cmdline.subImage.upper.x = std::atof(argv[++i]);
        cmdline.subImage.upper.y = std::atof(argv[++i]);
      }
      else if (arg == "-fovy" || arg == "--fov") {
        cmdline.camera.fovy = std::stof(argv[++i]);
      }
      else if (arg == "--camera") {
        cmdline.camera.vp.x = std::stof(argv[++i]);
        cmdline.camera.vp.y = std::stof(argv[++i]);
        cmdline.camera.vp.z = std::stof(argv[++i]);
        cmdline.camera.vi.x = std::stof(argv[++i]);
        cmdline.camera.vi.y = std::stof(argv[++i]);
        cmdline.camera.vi.z = std::stof(argv[++i]);
        cmdline.camera.vu.x = std::stof(argv[++i]);
        cmdline.camera.vu.y = std::stof(argv[++i]);
        cmdline.camera.vu.z = std::stof(argv[++i]);
      }
      else if (arg == "-win"  || arg == "--win" || arg == "--size") {
        cmdline.windowSize.x = std::atoi(argv[++i]);
        cmdline.windowSize.y = std::atoi(argv[++i]);
      }
      else if (arg == "-o") {
        cmdline.outFileName = argv[++i];
      }
      else if (arg == "-fps") {
        cmdline.fpsFileName = argv[++i];
      }
      else if (arg == "-spp" || arg == "--spp") {
        cmdline.spp = std::stoi(argv[++i]);
      }
      else if (arg == "--heat-map") {
        cmdline.heatMapEnabled = true;
        cmdline.heatMapScale = std::stof(argv[++i]);
      }
      else if (arg == "-dt") {
        cmdline.dt = std::stof(argv[++i]);
      }
      else if (arg == "--remap-from") {
        cmdline.xform.remap_from.lower.x = std::stof(argv[++i]);
        cmdline.xform.remap_from.lower.y = std::stof(argv[++i]);
        cmdline.xform.remap_from.lower.z = std::stof(argv[++i]);
        cmdline.xform.remap_from.upper.x = std::stof(argv[++i]);
        cmdline.xform.remap_from.upper.y = std::stof(argv[++i]);
        cmdline.xform.remap_from.upper.z = std::stof(argv[++i]);
      }
      else if (arg == "--remap-to") {
        cmdline.xform.remap_to.lower.x = std::stof(argv[++i]);
        cmdline.xform.remap_to.lower.y = std::stof(argv[++i]);
        cmdline.xform.remap_to.lower.z = std::stof(argv[++i]);
        cmdline.xform.remap_to.upper.x = std::stof(argv[++i]);
        cmdline.xform.remap_to.upper.y = std::stof(argv[++i]);
        cmdline.xform.remap_to.upper.z = std::stof(argv[++i]);
      }
      else if (arg == "--clip-plane") {
        static int currentPlane = 0;
        int P = currentPlane++;
        assert(P < CLIP_PLANES_MAX);
        cmdline.clipPlanes[P].enabled = true;
        cmdline.clipPlanes[P].N.x = std::stof(argv[++i]);
        cmdline.clipPlanes[P].N.y = std::stof(argv[++i]);
        cmdline.clipPlanes[P].N.z = std::stof(argv[++i]);
        cmdline.clipPlanes[P].d   = std::stof(argv[++i]);
      }
      else if (arg == "-rt") {
        cmdline.rendererType = std::atoi(argv[++i]);
      }
      else if (arg == "-sm") {
        cmdline.shadeMode = std::atoi(argv[++i]);
      }
      else if (arg == "--num-mcs") {
        cmdline.numMCs.x = std::atoi(argv[++i]);
        cmdline.numMCs.y = std::atoi(argv[++i]);
        cmdline.numMCs.z = std::atoi(argv[++i]);
      }
      else if (arg == "--light") {
        cmdline.lights[0].pos.x     = std::stof(argv[++i]);
        cmdline.lights[0].pos.y     = std::stof(argv[++i]);
        cmdline.lights[0].pos.z     = std::stof(argv[++i]);
        cmdline.lights[0].intensity = std::stof(argv[++i]);
        cmdline.lights[0].on = true;
      }
      else 
        usage("unknown cmdline arg '"+arg+"'");
    }
    
    OWLRenderer renderer(inFileName,
                         cmdline.gridsFileName,
                         cmdline.amrCellFileName,
                         cmdline.exaBrickFileName,
                         cmdline.bigMeshFileName,
                         cmdline.quickClustersFileName,
                         cmdline.meshFileName,
                         cmdline.scalarFileName,
                         cmdline.kdtreeFileName,
                         cmdline.majorantsFileName,
                         cmdline.xform.remap_from,
                         cmdline.xform.remap_to,
                         cmdline.numMCs);

    const box3f modelBounds = renderer.modelBounds;

    renderer.xf.colorMap = qtOWL::ColorMapLibrary().getMap(0);
    renderer.setColorMap(renderer.xf.colorMap);

    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      renderer.setClipPlane(i,
                            cmdline.clipPlanes[i].enabled,
                            cmdline.clipPlanes[i].N,
                            cmdline.clipPlanes[i].d);
    }

    renderer.setType((OWLRenderer::Type)cmdline.rendererType);
    renderer.setShadeMode(cmdline.shadeMode);
    if (!cmdline.lights[0].on) {
      cmdline.lights[0].pos = modelBounds.upper;
      cmdline.lights[0].intensity = 60.f;
    }
    renderer.setLightSource(0,cmdline.lights[0].pos,cmdline.lights[0].intensity,cmdline.lights[0].on);

    if (!cmdline.subImage.empty())
      renderer.setSubImage(cmdline.subImage,true);

#ifndef HEADLESS
    QApplication app(argc,argv);
    Viewer viewer(&renderer);

    viewer.enableFlyMode();

    viewer.enableInspectMode(qtOWL::OWLViewer::Arcball,modelBounds);
    //viewer.enableInspectMode(/* valid range of poi*/modelBounds,
    //                         /* min distance      */1e-3f,
    //                         /* max distance      */1e8f);

    if (cmdline.camera.vu != vec3f(0.f)) {
      viewer.setCameraOrientation(/*origin   */cmdline.camera.vp,
                                  /*lookat   */cmdline.camera.vi,
                                  /*up-vector*/cmdline.camera.vu,
                                  /*fovy(deg)*/cmdline.camera.fovy);
    } else {
      viewer.setCameraOrientation(/*origin   */
                                  modelBounds.center()
                                  + vec3f(-.3f, .7f, +1.f) * modelBounds.span(),
                                  /*lookat   */modelBounds.center(),
                                  /*up-vector*/vec3f(0.f, 1.f, 0.f),
                                  /*fovy(deg)*/70.f);
    }
    viewer.setWorldScale(1.1f*length(modelBounds.span()));
    viewer.lightInteractor.setWorldScale(length(modelBounds.span()));
    viewer.lightInteractor.setPos(cmdline.lights[0].pos);

    qtOWL::XFEditor *xfEditor = new qtOWL::XFEditor;
    range1f valueRange = renderer.valueRange;
    if (!cmdline.valueRange.is_empty())
      valueRange = cmdline.valueRange;

    QObject::connect(xfEditor,&qtOWL::XFEditor::colorMapChanged,
                     &viewer, &Viewer::colorMapChanged);
    QObject::connect(xfEditor,&qtOWL::XFEditor::rangeChanged,
                     &viewer, &Viewer::rangeChanged);
    QObject::connect(xfEditor,&qtOWL::XFEditor::opacityScaleChanged,
                     &viewer, &Viewer::opacityScaleChanged);
    QObject::connect(&viewer.lightInteractor,&LightInteractor::lightPosChanged,
                     &viewer, &Viewer::lightPosChanged);

    viewer.xfEditor = xfEditor;

    if (cmdline.xfFileName != "")
      xfEditor->loadFrom(cmdline.xfFileName);
    xfEditor->setAbsDomain(valueRange);
    viewer.opacityScaleChanged(xfEditor->getOpacityScale());

    QMainWindow guiWindow;
    QWidget *centralWidget = new QWidget;
    QVBoxLayout vlayout(centralWidget);

    vlayout.addWidget(xfEditor);

    // ==================================================================
    // Screenshot
    // ==================================================================

    QGroupBox *screenShotBox = new QGroupBox("Screen Shot");
    vlayout.addWidget(screenShotBox);

    QHBoxLayout screenShotLayout(screenShotBox);
    QPushButton screenShotNowButton("Now");
    screenShotLayout.addWidget(&screenShotNowButton);

    QHBoxLayout screenShotFramesLayout;
    QFrame line;
    line.setFrameShape(QFrame::VLine);
    line.setFrameShadow(QFrame::Sunken);
    QLabel screenShotFramesLabel("# Frames:");
    QSpinBox screenShotFramesSpinBox;
    QPushButton screenShotFramesButton("Schedule...");
    screenShotFramesLayout.addWidget(&line);
    screenShotFramesLayout.addWidget(&screenShotFramesLabel);
    screenShotFramesLayout.addWidget(&screenShotFramesSpinBox);
    screenShotFramesLayout.addWidget(&screenShotFramesButton);
    screenShotLayout.addLayout(&screenShotFramesLayout);

    QHBoxLayout screenShotSecondsLayout;
    QLabel screenShotSecondsLabel("Seconds:");
    QDoubleSpinBox screenShotSecondsSpinBox;
    QPushButton screenShotSecondsButton("Schedule...");
    screenShotSecondsLayout.addWidget(&line);
    screenShotSecondsLayout.addWidget(&screenShotSecondsLabel);
    screenShotSecondsLayout.addWidget(&screenShotSecondsSpinBox);
    screenShotSecondsLayout.addWidget(&screenShotSecondsButton);
    screenShotLayout.addLayout(&screenShotSecondsLayout);

    // ==================================================================
    // Rendering settings
    // ==================================================================

    QGroupBox *renderSettingsBox = new QGroupBox("Render Settings");
    vlayout.addWidget(renderSettingsBox);

    // Renderer type
    QHBoxLayout rendererTypeLayout;
    QLabel rendererTypeLabel("Renderer: ");
    QComboBox rendererTypeSelection;
    rendererTypeSelection.addItem("Path Tracer");
    rendererTypeSelection.addItem("Direct Lighting Renderer");
    rendererTypeSelection.addItem("A+E Ray Marcher");
    rendererTypeSelection.setCurrentIndex(cmdline.rendererType);
    rendererTypeLayout.addWidget(&rendererTypeLabel);
    rendererTypeLayout.addWidget(&rendererTypeSelection);


    // Shade mode
    QHBoxLayout shadeModeLayout;
    QLabel shadeModeLabel("Shade Mode: ");
    QComboBox shadeModeSelection;
    for (int i=0; i<renderer.shadeModes().size(); ++i) {
      shadeModeSelection.addItem(renderer.shadeModes()[i].c_str());
    }
    shadeModeSelection.setCurrentIndex(cmdline.shadeMode);
    shadeModeLayout.addWidget(&shadeModeLabel);
    shadeModeLayout.addWidget(&shadeModeSelection);

    // Light source
    QHBoxLayout light0Layout;
    QCheckBox light0Enabled("Enable Light");
    light0Enabled.setCheckState(Qt::Unchecked);
    QCheckBox editLightEnabled("Edit Light Pos");
    editLightEnabled.setCheckState(Qt::Unchecked);
    QLabel light0IntensityLabel("Intensity");
    QDoubleSpinBox light0IntensityBox;
    light0IntensityBox.setMinimum(0.);
    light0IntensityBox.setMaximum(999.);
    light0IntensityBox.setValue(cmdline.lights[0].intensity);
    QComboBox light0UnitsSelection;
    light0UnitsSelection.addItem("W");
    light0UnitsSelection.addItem("KW");
    light0UnitsSelection.addItem("MW");
    light0UnitsSelection.addItem("GW");
    const float lscale = 1.f/viewer.renderer->lightSpaceTransform.l.vx.x;
    int item = log10(lscale)/3;
    light0UnitsSelection.setCurrentIndex(item);
    light0Layout.addWidget(&light0Enabled);
    light0Layout.addWidget(&editLightEnabled);
    light0Layout.addWidget(&light0IntensityLabel);
    light0Layout.addWidget(&light0IntensityBox);
    light0Layout.addWidget(&light0UnitsSelection);

    // Add layouts
    QVBoxLayout settingsvlayout(renderSettingsBox);
    settingsvlayout.addLayout(&rendererTypeLayout);
    settingsvlayout.addLayout(&shadeModeLayout);
    settingsvlayout.addLayout(&light0Layout);

    // ==================================================================
    // Sub image
    // ==================================================================

    QGroupBox *subImageBox = new QGroupBox("Sub-image selection");
    vlayout.addWidget(subImageBox);

    QCheckBox *subImageSelectionEnabled = new QCheckBox("Viewport selection enabled");
    subImageSelectionEnabled->setCheckState(Qt::Unchecked);

    QPushButton *undoAllSubImageButton = new QPushButton("undo all");
    QPushButton *undoSubImageButton = new QPushButton("undo");
    QPushButton *redoSubImageButton = new QPushButton("redo");
    QPushButton *redoAllSubImageButton = new QPushButton("redo all");

    QLabel *subImageLabel = new QLabel("");
    subImageLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    QRadioButton *subImageKeepAspect = new QRadioButton("Keep Aspect");
    QRadioButton *subImageSquare = new QRadioButton("Square");
    QRadioButton *subImageFree = new QRadioButton("Free");
    subImageKeepAspect->setChecked(true);

    QVBoxLayout subimgvlayout(subImageBox);
    QHBoxLayout subimgcontraintlayout;
    QHBoxLayout subimgundolayout;
    QHBoxLayout subimglabellayout;
    subimgvlayout.addWidget(subImageSelectionEnabled);
    subimgvlayout.addLayout(&subimgcontraintlayout);
    subimgvlayout.addLayout(&subimgundolayout);
    subimgvlayout.addLayout(&subimglabellayout);

    subimgcontraintlayout.addWidget(subImageKeepAspect);
    subimgcontraintlayout.addWidget(subImageSquare);
    subimgcontraintlayout.addWidget(subImageFree);

    subimgundolayout.addWidget(undoAllSubImageButton);
    subimgundolayout.addWidget(undoSubImageButton);
    subimgundolayout.addWidget(redoSubImageButton);
    subimgundolayout.addWidget(redoAllSubImageButton);
    subimglabellayout.addWidget(subImageLabel);

    // ==================================================================
    // Clip planes
    // ==================================================================

    QGroupBox *clipPlaneBox = new QGroupBox("Clip Plane");
    vlayout.addWidget(clipPlaneBox);

    QComboBox *clipPlaneSelection = new QComboBox;
    clipPlaneSelection->addItem("Clip Plane 0");
    clipPlaneSelection->addItem("Clip Plane 1");

    QCheckBox *clipPlaneEnabled = new QCheckBox("enabled");
    clipPlaneEnabled->setCheckState(cmdline.clipPlanes[g_clipPlaneSelected].enabled
                                          ? Qt::Checked : Qt::Unchecked);

    QVBoxLayout clipvlayout(clipPlaneBox);
    QHBoxLayout clipSelectionLayout;
    clipvlayout.addLayout(&clipSelectionLayout);
    clipSelectionLayout.addWidget(clipPlaneSelection);
    clipSelectionLayout.addWidget(clipPlaneEnabled);

    QVBoxLayout clipAdjustLayout;
    QHBoxLayout clipOffsetLayout;
    QHBoxLayout clipNormalLayout;

    auto  worldToOffset = [=](float d, const vec3f N) {
      float f0 = dot(modelBounds.lower,N);
      float f1 = dot(modelBounds.upper,N);
      return (d-f0)/(f1-f0);
    };

    QLabel offset("Offset:");
    QSlider *clipOff = new QSlider(Qt::Horizontal);
    clipOff->setEnabled(cmdline.clipPlanes[g_clipPlaneSelected].enabled);
    clipOff->setValue(worldToOffset(cmdline.clipPlanes[g_clipPlaneSelected].d,
                                    cmdline.clipPlanes[g_clipPlaneSelected].N)*100);
    clipOffsetLayout.addWidget(&offset);
    clipOffsetLayout.addWidget(clipOff);

    const vec3f N = cmdline.clipPlanes[g_clipPlaneSelected].N;

    QLabel nx("N.x:");
    QLabel ny("N.y:");
    QLabel nz("N.z:");
    QSlider *nxSlider = new QSlider(Qt::Horizontal);
    QSlider *nySlider = new QSlider(Qt::Horizontal);
    QSlider *nzSlider = new QSlider(Qt::Horizontal);
    nxSlider->setEnabled(cmdline.clipPlanes[g_clipPlaneSelected].enabled);
    nySlider->setEnabled(cmdline.clipPlanes[g_clipPlaneSelected].enabled);
    nzSlider->setEnabled(cmdline.clipPlanes[g_clipPlaneSelected].enabled);
    nxSlider->setMinimum(-100);
    nySlider->setMinimum(-100);
    nzSlider->setMinimum(-100);
    nxSlider->setValue(N.x*100);
    nySlider->setValue(N.y*100);
    nzSlider->setValue(N.z*100);

    clipNormalLayout.addWidget(&nx);
    clipNormalLayout.addWidget(nxSlider);
    clipNormalLayout.addWidget(&ny);
    clipNormalLayout.addWidget(nySlider);
    clipNormalLayout.addWidget(&nz);
    clipNormalLayout.addWidget(nzSlider);

    clipAdjustLayout.addLayout(&clipOffsetLayout);
    clipAdjustLayout.addLayout(&clipNormalLayout);

    clipvlayout.addLayout(&clipAdjustLayout);

    clipPlaneBox->setLayout(&clipvlayout);

    // Connections
    auto  offsetToWorld = [=](float d01, const vec3f N) {
      float f0 = dot(modelBounds.lower,N);
      float f1 = dot(modelBounds.upper,N);
      return (1.f-d01) * f0 + d01*f1;
    };

    // Screen shot now
    QObject::connect(&screenShotNowButton, &QPushButton::pressed,
      [&]() {
        viewer.scheduleScreenShot(0);
      });

    // Screen shot after frames
    QObject::connect(&screenShotFramesButton, &QPushButton::pressed,
      [&]() {
        viewer.scheduleScreenShot(screenShotFramesSpinBox.value());
      });

    // Screen shot after seconds
    QObject::connect(&screenShotSecondsButton, &QPushButton::pressed,
      [&]() {
        viewer.scheduleScreenShot(screenShotSecondsSpinBox.value());
      });

    // Renderer type select
    QObject::connect(&rendererTypeSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        renderer.setType((OWLRenderer::Type)item);
      });

    // Shade mode select
    QObject::connect(&shadeModeSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        renderer.setShadeMode(item);
      });

    // Light pos editing enabled
    QObject::connect(&editLightEnabled, qOverload<int>(&QCheckBox::stateChanged),
      [&](int state) {
        viewer.lightInteractor.toggleActive();
      });

    // Light intensity
    QObject::connect(&light0IntensityBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
      [&](double value) {
        cmdline.lights[0].intensity = value;
        viewer.renderer->setLightSource(0,cmdline.lights[0].pos,
                                        cmdline.lights[0].intensity,
                                        cmdline.lights[0].on);
      });

    // Light intensity units
    QObject::connect(&light0UnitsSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        float val = powf(1000.f,float(item));
        affine3f xform;
        xform = xform.scale(1.f/vec3f(val));
        viewer.renderer->setLightSpaceTransform(xform);
      });

    // Light pos editing toggled via key press
    QObject::connect(&viewer, &Viewer::lightEdittingToggled, &editLightEnabled, &QCheckBox::toggle);

    // light0 enabled
    QObject::connect(&light0Enabled, qOverload<int>(&QCheckBox::stateChanged),
      [&](int state) {
        cmdline.lights[0].on = state;
        viewer.renderer->setLightSource(0,cmdline.lights[0].pos,
                                        cmdline.lights[0].intensity,
                                        cmdline.lights[0].on);
      });

    // Clip plane select
    QObject::connect(clipPlaneSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        g_clipPlaneSelected = item;
      });

    // Sub image enable
    QObject::connect(subImageSelectionEnabled, qOverload<int>(&QCheckBox::stateChanged),
      [&](int state) {
        viewer.subImageSelecting = state;
      });

    // Sub image keep aspect
    QObject::connect(subImageKeepAspect, &QRadioButton::toggled,
      [&](bool checked) {
        if (checked) viewer.subImageSelectionContraint = Viewer::SubImageSelectionContraint::KeepAspect;
      });

    // Sub image square
    QObject::connect(subImageSquare, &QRadioButton::toggled,
      [&](bool checked) {
        if (checked) viewer.subImageSelectionContraint = Viewer::SubImageSelectionContraint::Square;
      });

    // Sub image free
    QObject::connect(subImageFree, &QRadioButton::toggled,
      [&](bool checked) {
        if (checked) viewer.subImageSelectionContraint = Viewer::SubImageSelectionContraint::Free;
      });

    // Sub image selection undo all
    QObject::connect(undoAllSubImageButton, &QPushButton::pressed,
      [&]() {
        viewer.subImageUndoStackTop = 0;
        renderer.setSubImage({},false); // not active
      });

    // Sub image selection undo last
    QObject::connect(undoSubImageButton, &QPushButton::pressed,
      [&]() {
        viewer.subImageUndoStackTop = std::max<int>(0,viewer.subImageUndoStackTop-1);
        box2f subImg = viewer.computeSubImageUV();
        renderer.setSubImage(subImg,viewer.subImageUndoStackTop>0);
      });

    // Sub image selection redo last
    QObject::connect(redoSubImageButton, &QPushButton::pressed,
      [&]() {
        viewer.subImageUndoStackTop = std::min<int>(viewer.subImageUndoStackTop+1,
                                                    (int32_t)viewer.subImageUndoStack.size());
        box2f subImg = viewer.computeSubImageUV();
        renderer.setSubImage(subImg,viewer.subImageUndoStackTop>0);
      });

    // Sub image selection redo all
    QObject::connect(redoAllSubImageButton, &QPushButton::pressed,
      [&]() {
        viewer.subImageUndoStackTop = (int)viewer.subImageUndoStack.size();
        box2f subImg = viewer.computeSubImageUV();
        renderer.setSubImage(subImg,viewer.subImageUndoStackTop>0);
      });

    // Sub image changed from within the viewer
    QObject::connect(&viewer, &Viewer::subImageChanged,
      [&](box2f subImageUV) {
        std::stringstream ss;
        ss << std::fixed;
        ss << std::setprecision(3);
        ss << "Cmdline: \"--subimg " << subImageUV.lower.x << ' '
                                     << subImageUV.lower.y << ' '
                                     << subImageUV.upper.x << ' '
                                     << subImageUV.upper.y << '\"';
        std::string str = ss.str();
        subImageLabel->setText(str.c_str());
        std::cout << str << '\n';
      });

    // Sub image selection changed from within the viewer
    QObject::connect(&viewer, &Viewer::subImageSelectionChanged,
      [&](box2f subImageUV) {
        const vec2f ws(viewer.getWindowSize());
        box2i subImageWin(clamp(vec2i(subImageUV.lower*ws),vec2i(0),viewer.getWindowSize()-1),
                          clamp(vec2i(subImageUV.upper*ws),vec2i(0),viewer.getWindowSize()-1));
        std::stringstream ss;
        ss << "Selection: " << subImageWin << ", selection size: " << subImageWin.size();
        std::string str = ss.str();
        subImageLabel->setText(str.c_str());
      });

    // Clip plane enable
    QObject::connect(clipPlaneEnabled, qOverload<int>(&QCheckBox::stateChanged),
      [&](int state) {
        clipOff->setEnabled(state);
        nxSlider->setEnabled(state);
        nySlider->setEnabled(state);
        nzSlider->setEnabled(state);
        if (g_clipPlaneSelected >= 0) {
          const vec3f N = normalize(vec3f(nxSlider->sliderPosition()/100.f,
                                          nySlider->sliderPosition()/100.f,
                                          nzSlider->sliderPosition()/100.f));
          const float d = offsetToWorld(clipOff->sliderPosition()/100.f,N);
          renderer.setClipPlane(g_clipPlaneSelected,
                                state,N,d);
          cmdline.clipPlanes[g_clipPlaneSelected].N = N;
          cmdline.clipPlanes[g_clipPlaneSelected].d = d;
        }

        cmdline.clipPlanes[g_clipPlaneSelected].enabled = state;
        cmdline.clipPlanes[g_clipPlaneSelected].show = state;
      });

    // Clip plane sliders and dials
    QObject::connect(clipOff, &QSlider::sliderMoved,
      [&](int position) {
        if (g_clipPlaneSelected >= 0) {
          const vec3f N = normalize(vec3f(nxSlider->sliderPosition()/100.f,
                                          nySlider->sliderPosition()/100.f,
                                          nzSlider->sliderPosition()/100.f));
          const float d = offsetToWorld(position/100.f,N);
          renderer.setClipPlane(g_clipPlaneSelected,
                                clipPlaneEnabled->isChecked(),N,d);
          cmdline.clipPlanes[g_clipPlaneSelected].N = N;
          cmdline.clipPlanes[g_clipPlaneSelected].d = d;
        }
      });

    QObject::connect(nxSlider, &QSlider::sliderMoved,
      [&](int position) {
        if (g_clipPlaneSelected >= 0) {
          const vec3f N = normalize(vec3f(position/100.f,
                                          nySlider->sliderPosition()/100.f,
                                          nzSlider->sliderPosition()/100.f));
          const float d = offsetToWorld(clipOff->sliderPosition()/100.f,N);
          renderer.setClipPlane(g_clipPlaneSelected,
                                clipPlaneEnabled->isChecked(),N,d);
          cmdline.clipPlanes[g_clipPlaneSelected].N = N;
          cmdline.clipPlanes[g_clipPlaneSelected].d = d;
        }
      });

    QObject::connect(nySlider, &QSlider::sliderMoved,
      [&](int position) {
        if (g_clipPlaneSelected >= 0) {
          const vec3f N = normalize(vec3f(nxSlider->sliderPosition()/100.f,
                                          position/100.f,
                                          nzSlider->sliderPosition()/100.f));
          const float d = offsetToWorld(clipOff->sliderPosition()/100.f,N);
          renderer.setClipPlane(g_clipPlaneSelected,
                                clipPlaneEnabled->isChecked(),N,d);
          cmdline.clipPlanes[g_clipPlaneSelected].N = N;
          cmdline.clipPlanes[g_clipPlaneSelected].d = d;
        }
      });

    QObject::connect(nzSlider, &QSlider::sliderMoved,
      [&](int position) {
        if (g_clipPlaneSelected >= 0) {
          const vec3f N = normalize(vec3f(nxSlider->sliderPosition()/100.f,
                                          nySlider->sliderPosition()/100.f,
                                          position/100.f));
          const float d = offsetToWorld(clipOff->sliderPosition()/100.f,N);
          renderer.setClipPlane(g_clipPlaneSelected,
                                clipPlaneEnabled->isChecked(),N,d);
          cmdline.clipPlanes[g_clipPlaneSelected].N = N;
          cmdline.clipPlanes[g_clipPlaneSelected].d = d;
        }
      });

    guiWindow.setCentralWidget(centralWidget);

    guiWindow.show();
    viewer.show();

    app.exec();
#else
    Viewer viewer(&renderer);
    viewer.setOutFileName(cmdline.outFileName);
    viewer.setOutFileNameFPS(cmdline.fpsFileName);
    viewer.resize(cmdline.windowSize);

    if (cmdline.camera.vu != vec3f(0.f)) {
      viewer.setCameraOrientation(/*origin   */cmdline.camera.vp,
                                  /*lookat   */cmdline.camera.vi,
                                  /*up-vector*/cmdline.camera.vu,
                                  /*fovy(deg)*/cmdline.camera.fovy);
    } else {
      viewer.setCameraOrientation(/*origin   */
                                  modelBounds.center()
                                  + vec3f(-.3f, .7f, +1.f) * modelBounds.span(),
                                  /*lookat   */modelBounds.center(),
                                  /*up-vector*/vec3f(0.f, 1.f, 0.f),
                                  /*fovy(deg)*/70.f);
    }
    viewer.setWorldScale(1.1f*length(modelBounds.span()));
    viewer.lightInteractor.setWorldScale(length(modelBounds.span()));
    viewer.lightInteractor.setPos(cmdline.lights[0].pos);

    if (cmdline.xfFileName != "") {
      viewer.loadTransferFunction(cmdline.xfFileName);
      renderer.setColorMap(viewer.xf.colorMap);
      viewer.xf.absDomain = renderer.valueRange;

      range1f r = viewer.xfRange();

      float lo = min(r.lower,r.upper);
      float hi = max(r.lower,r.upper);
      renderer.setRange({lo,hi});
      renderer.setRelDomain(viewer.xf.relDomain);

      renderer.setOpacityScale(viewer.xf.opacityScale);

      renderer.resetAccum();
    }

    viewer.run();
    return 0;
#endif
  }
} // ::exa

#include "viewer.moc"
// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

