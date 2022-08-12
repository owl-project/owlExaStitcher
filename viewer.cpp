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

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include "qtOWL/OWLViewer.h"
#include "qtOWL/XFEditor.h"
#include "OWLRenderer.h"

#define DUMP_FRAMES 0

static int g_clipPlaneSelected = 0;

namespace exa {
  using qtOWL::Camera;
  using qtOWL::SimpleCamera;

  struct {
    std::string scalarFileName = "";
    std::string gridsFileName = "";
    std::string amrCellFileName = "";
    std::string meshFileName = "";
    std::string xfFileName = "";
    std::string outFileName = "owlDVR.png";
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
    int shadeMode = 0;
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

  struct Viewer : public qtOWL::OWLViewer {
    typedef qtOWL::OWLViewer inherited;

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
        std::cout << "saving screenshot to 'Witcher3.png'" << std::endl;
        screenShot("Witcher3.png");
        break;
      case 'H':
        //renderer->heatMapEnabled = !renderer->heatMapEnabled;
        break;
      case '<':
        //renderer->heatMapScale /= 1.5f;
        break;
      case '>':
        //renderer->heatMapScale *= 1.5f;
        break;
      case ')':
        //renderer->spp++;
        //PRINT(renderer->spp);
        break;
      case '(':
        //renderer->spp = max(1,renderer->spp-1);
        //PRINT(renderer->spp);
        break;
      case 'T':
        if (xfEditor) xfEditor->saveTo("owlDVR.xf");
        break;
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

  public slots:
    void colorMapChanged(qtOWL::XFEditor *xf);
    void rangeChanged(range1f r);
    void opacityScaleChanged(double scale);

  public:

    OWLRenderer *const renderer;
    qtOWL::XFEditor *xfEditor = nullptr;
  };

  void Viewer::resize(const vec2i &newSize) 
  {
    // ... tell parent to resize (also resizes the pbo in the wingdow)
    inherited::resize(newSize);
    cameraChanged();
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
    Mat4 view = lookAt(camera.getFrom(),
                       camera.getAt(),
                       camera.getUp());
    Mat4 proj = perspective(camera.getFovyInDegrees()*M_PI/180.f,
                            float(fbSize.x)/fbSize.y,
                            .001f,100000.f);

    
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
      else if (arg == "-mesh") {
        cmdline.meshFileName = argv[++i];
      }
      else if (arg == "-xf") {
        cmdline.xfFileName = argv[++i];
      }
      else if (arg == "-fovy") {
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
      else if (arg == "-sm") {
        cmdline.shadeMode = std::atoi(argv[++i]);
      }
      else
        usage("unknown cmdline arg '"+arg+"'");
    }
    
    if (inFileName == "" && cmdline.amrCellFileName == "")
      usage("no filename specified");

    OWLRenderer renderer(inFileName,
                         cmdline.gridsFileName,
                         cmdline.amrCellFileName,
                         cmdline.meshFileName,
                         cmdline.scalarFileName);

    const box3f modelBounds = renderer.modelBounds;

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

    renderer.xf.colorMap = qtOWL::ColorMapLibrary().getMap(0);
    renderer.setColorMap(renderer.xf.colorMap);

    for (int i=0; i<CLIP_PLANES_MAX; ++i) {
      renderer.setClipPlane(i,
                            cmdline.clipPlanes[i].enabled,
                            cmdline.clipPlanes[i].N,
                            cmdline.clipPlanes[i].d);
    }

    renderer.setShadeMode(cmdline.shadeMode);

    qtOWL::XFEditor *xfEditor = new qtOWL::XFEditor;
    range1f valueRange = renderer.valueRange;

    QObject::connect(xfEditor,&qtOWL::XFEditor::colorMapChanged,
                     &viewer, &Viewer::colorMapChanged);
    QObject::connect(xfEditor,&qtOWL::XFEditor::rangeChanged,
                     &viewer, &Viewer::rangeChanged);
    QObject::connect(xfEditor,&qtOWL::XFEditor::opacityScaleChanged,
                     &viewer, &Viewer::opacityScaleChanged);

    viewer.xfEditor = xfEditor;

    if (cmdline.xfFileName != "")
      xfEditor->loadFrom(cmdline.xfFileName);
    xfEditor->setAbsDomain(valueRange);

    QMainWindow guiWindow;
    QWidget *centralWidget = new QWidget;
    QVBoxLayout vlayout(centralWidget);

    vlayout.addWidget(xfEditor);

    QGroupBox *renderSettingsBox = new QGroupBox("Render Settings");
    vlayout.addWidget(renderSettingsBox);

    QComboBox *shadeModeSelection = new QComboBox;
    for (int i=0; i<renderer.shadeModes().size(); ++i) {
      shadeModeSelection->addItem(renderer.shadeModes()[i].c_str());
    }
    shadeModeSelection->setCurrentIndex(cmdline.shadeMode);

    QVBoxLayout settingsvlayout(renderSettingsBox);
    settingsvlayout.addWidget(shadeModeSelection);

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

    // Shade mode select
    QObject::connect(shadeModeSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        renderer.setShadeMode(item);
      });

    // Clip plane select
    QObject::connect(clipPlaneSelection, qOverload<int>(&QComboBox::currentIndexChanged),
      [&](int item) {
        g_clipPlaneSelected = item;
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
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

