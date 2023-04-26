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
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl2.h>
#include <qtOWL/ColorMaps.h>
#include "samples/common/owlViewer/InspectMode.h"
#include "samples/common/owlViewer/OWLViewer.h"
#include "OWLRenderer.h"
#include "ImGuiCUDAGLWidget.h"
#include "VolumeLines.h"
#include "model/ExaBrickModel.h"
#include "TFEditor.h"

// #include "YueKDTree.h"
// #include "YueVolume.h"


#define DUMP_FRAMES 0

static int g_clipPlaneSelected = 0;

namespace exa {
  using owl::viewer::Camera;
  using owl::viewer::SimpleCamera;

  struct {
    std::string scalarFileName0 = "";
    std::string scalarFileName1 = "";
    std::string scalarFileName2 = "";
    std::string scalarFileName3 = "";
    std::string scalarFileName4 = "";
    std::string scalarFileName5 = "";
    std::string scalarFileName6 = "";
    std::string scalarFileName7 = "";
    std::string gridsFileName = "";
    std::string amrCellFileName = "";
    std::string exaBrickFileName = "";
    std::string bigMeshFileName = "";
    std::string quickClustersFileName = "";
    std::string kdtreeFileName = "";
    std::string majorantsFileName = "";
    std::string meshFileName = "";
    std::string xfFileName[FIELDS_MAX];
    std::string outFileName = "Witcher3.png";
    std::string fpsFileName = "fps.log";
    range1f valueRange = {1e30f,-1e30f};
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

  struct Viewer : public owl::viewer::OWLViewer {
    typedef owl::viewer::OWLViewer inherited;

  public:
    Viewer(OWLRenderer *renderer)
      : inherited("exastitch", cmdline.windowSize)
      , renderer(renderer)
      , imguiInitialized(false)
    {
    }

    bool initImGui()
    {
      IMGUI_CHECKVERSION();
      ImGui::CreateContext();

      ImGuiIO& io = ImGui::GetIO();

      //io.Fonts->AddFontFromFileTTF(
      //    fontName.c_str(),
      //    24.0f * highDPIscaleFactor,
      //    NULL,
      //    NULL
      //    );
      //setImGuiStyle(highDPIscaleFactor);

      // setup platform/renderer bindings
      if (!ImGui_ImplGlfw_InitForOpenGL(inherited::handle, true)) { return false; }
      if (!ImGui_ImplOpenGL2_Init()) { return false; }

      return true;
    }

    /*! this function gets called whenever the viewer widget changes
      camera settings */
    void cameraChanged() override;
    void resize(const vec2i &newSize) override;
    /*! gets called whenever the viewer needs us to re-render out widget */
    void render() override;
      
    /*! draw framebuffer using OpenGL */
    void draw();

    void special(int key, int mods, const vec2i &where) override{
      ImGuiIO& io = ImGui::GetIO();

      if (io.WantTextInput || io.WantCaptureKeyboard) {
        return;
      }

      inherited::special(key,mods, where);
    };

    /*! this gets called when the user presses a key on the keyboard ... */
    void key(char key, const vec2i &where) override
    {
      ImGuiIO& io = ImGui::GetIO();

      if (io.WantTextInput || io.WantCaptureKeyboard) {
        return;
      }

      inherited::key(key,where);
      renderer->resetAccum();
      switch (key) {
      case '!':
        std::cout << "saving screenshot to '" << cmdline.outFileName << ".png'\n";
        screenShot(cmdline.outFileName+".png");
        exit(0);
        break;
      case 'H':
        renderer->heatMapEnabled = !renderer->heatMapEnabled;
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
        //if (xfEditor) xfEditor->saveTo("owlDVR.xf");
        break;
      // case 'Y': {
      //   if (auto model = renderer->model->as<ExaBrickModel>()) {
      //     std::string baseFileName = cmdline.scalarFileName;
      //     std::string cmFileName = baseFileName+"_MajorantColorMap.bin";
      //     std::string domainsFileName = baseFileName+"_MajorantDomains.bin";

      //     std::vector<float> rgbaCM(xfEditor->getColorMap().size()*4);
      //     memcpy(rgbaCM.data(),xfEditor->getColorMap().data(),
      //            rgbaCM.size()*sizeof(rgbaCM[0]));

      //     YueVolume vol(model,&rgbaCM,renderer->xf.absDomain,renderer->xf.relDomain);
      //     volkd::KDTree kdtree(vol,&rgbaCM);

      //     std::vector<std::pair<box3f,float>> domains;
      //     while (!kdtree.nodes.empty()) {
      //       volkd::Node node = kdtree.nodes.top();
      //       kdtree.nodes.pop();
      //       range1f tfRange = vol.min_max(node.domain,&rgbaCM);
      //       std::cout << "Domain " << domains.size() << ": "
      //                 << node.domain << ", majorant: " << tfRange.upper << '\n';
      //       domains.push_back({node.domain,tfRange.upper});
      //     }

      //     std::ofstream cmFile(cmFileName, std::ios::binary);
      //     uint64_t cmSize = rgbaCM.size();
      //     cmFile.write((char *)&cmSize,sizeof(cmSize));
      //     cmFile.write((char *)rgbaCM.data(),rgbaCM.size()*sizeof(rgbaCM[0]));

      //     std::ofstream domainsFile(domainsFileName, std::ios::binary);
      //     uint64_t numDomains = domains.size();
      //     domainsFile.write((char *)&numDomains,sizeof(numDomains));
      //     domainsFile.write((char *)domains.data(),domains.size()*sizeof(domains[0]));
      //   }
      //   break;
      // }
      // case 'y': {
      //   if (auto model = renderer->model->as<ExaBrickModel>()) {
      //     std::string baseFileName = cmdline.scalarFileName;
      //     std::string cmFileName = baseFileName+"_MajorantColorMap.bin";
      //     std::string domainsFileName = baseFileName+"_MajorantDomains.bin";

      //     std::vector<float> rgbaCM(xfEditor->getColorMap().size()*4);
      //     memcpy(rgbaCM.data(),xfEditor->getColorMap().data(),
      //            rgbaCM.size()*sizeof(rgbaCM[0]));

      //     std::ifstream domainsFileIN(domainsFileName, std::ios::binary);
      //     uint64_t numDomains = 0;
      //     domainsFileIN.read((char *)&numDomains,sizeof(numDomains));
      //     std::vector<std::pair<box3f,float>> domains(numDomains);
      //     domainsFileIN.read((char *)domains.data(),domains.size()*sizeof(domains[0]));

      //     YueVolume vol(model,&rgbaCM,renderer->xf.absDomain,renderer->xf.relDomain);

      //     std::vector<std::pair<box3f,float>> newDomains;
      //     for (size_t i=0; i<domains.size(); ++i) {
      //       range1f tfRange = vol.min_max(domains[i].first,&rgbaCM);
      //       std::cout << "Domain " << i << ": " << domains[i].first
      //                 << ", majorant: " << tfRange.upper << '\n';
      //       newDomains.push_back({domains[i].first,tfRange.upper});
      //     }

      //     std::ofstream cmFile(cmFileName, std::ios::binary);
      //     uint64_t cmSize = rgbaCM.size();
      //     cmFile.write((char *)&cmSize,sizeof(cmSize));
      //     cmFile.write((char *)rgbaCM.data(),rgbaCM.size()*sizeof(rgbaCM[0]));

      //     std::ofstream domainsFile(domainsFileName, std::ios::binary);
      //     domainsFile.write((char *)&numDomains,sizeof(numDomains));
      //     domainsFile.write((char *)newDomains.data(),newDomains.size()*sizeof(newDomains[0]));
      //   }
      //   break;
      // }
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

    void mouseMotion(const vec2i &where)
    {
      ImGuiIO& io = ImGui::GetIO();
      //io.MousePos = ImVec2((float)where.x, (float)where.y);
      if (io.WantCaptureMouse) {
        return;
      }
      inherited::mouseMotion(where);
    }

//    void mouseButtonLeft(const vec2i &where, bool pressed)
//    {
//      ImGuiIO& io = ImGui::GetIO();
//      io.MousePos = ImVec2((float)where.x, (float)where.y);
//      int imgui_button = 0; // left (right is 1, middle is 2)
//      io.MouseDown[imgui_button] = pressed;
//      inherited::mouseButtonLeft(where, pressed);
//    }

//    void mouseDragLeft(const vec2i &where, const vec2i &delta)
//    {
//      ImGuiIO& io = ImGui::GetIO();
//      io.MousePos = ImVec2((float)where.x, (float)where.y);
//      int imgui_button = 0; // left (right is 1, middle is 2)
//      inherited::mouseDragLeft(where, delta);
//    }

  public:

    OWLRenderer *const renderer;
    //qtOWL::XFEditor *xfEditor = nullptr;

    bool imguiInitialized;

    vec2i downPos { 0, 0 };
    box2f lastSubImageWin;
    double screenShotAfterSec = -1.0;
    double seconds = 0.0;
    int screenShotAfterFrames = -1;
    int frames = 0;

    //vkt::LookupTable vktLUT;
    //vkt::TransfuncEditor tfe;
    TFEditor tfe[FIELDS_MAX];
    ImGuiCUDAGLWidget volumeLineWidget;
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

    t_last = t_now;

#if DUMP_FRAMES
    // just dump the 10th frame, then hard-exit
    static int g_frameID = 0;
    // if (g_frameID % 16 == 0) {
    //   std::cout << "g_frameID: " << g_frameID << std::endl;
    // }

    if (g_frameID++ >= 256) {
      screenShot(cmdline.outFileName+".png");
      exit(0);
    }
#endif
  }

  /*! draw framebuffer using OpenGL */
  void Viewer::draw()
  {
    inherited::draw();

    if (!imguiInitialized){
      if (!initImGui()) {
        std::cerr << "failed to initialize Dear ImGui!\n";
        exit(EXIT_FAILURE); // not much use in carrying on then..
      }
      imguiInitialized = true;
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("TFE");
    // !!!Temporary!!!
    static int fieldID = 0;
    const int numFields = renderer->getNumFields();
    int fid = fieldID;

    ImGui::SliderInt("FieldID", &fid, 0, numFields - 1);

    if (fid != fieldID){
      fieldID = fid;
      renderer->setActiveField(fieldID);
    }

    tfe[fieldID].drawImmediate();
    ImGui::End();

    // volume line widget
    static VolumeLines vl;
    static float minImportance = 0.025f;
    static float P = 1.f; // volume lines exponent
    static bool normalize = false; // normalize volume lines so they occupy the full height
    static bool globalColors = false;
    static int cmID = -1;
    static bool showSettings = false;
    static bool first=true;
    static VolumeLines::Mode mode = VolumeLines::Lines;
    if (renderer->model && renderer->model->as<ExaBrickModel>()) {
      if (first) {
      volumeLineWidget.resize(1024,128);
      vl.reset(renderer->model->as<ExaBrickModel>());
      volumeLineWidget.setPassiveMotionFunc([&](int x, int y, int button) { vl.onMouseMove(x,y,button); });
      volumeLineWidget.setMotionFunc([&](int x, int y, int button) { vl.onMouseDrag(x,y,button); });
      volumeLineWidget.setPressFunc([&](int x, int y, int button) { vl.onMousePress(x,y,button); });
      volumeLineWidget.setReleaseFunc([&](int x, int y, int button) { vl.onMouseRelease(x,y,button); });
      first=false;
      }
      auto surf = volumeLineWidget.map();
      vl.draw(surf,volumeLineWidget.width(),volumeLineWidget.height());
      volumeLineWidget.unmap();

      // VL widget
      ImGui::Begin("Volume Line(s)");
      volumeLineWidget.drawImmediate();
      //ImGui::SameLine(ImGui::GetWindowWidth()-30);
      ImGui::Spacing();
      if (ImGui::Button("Settings...")) {
        showSettings = true;
      }
      ImGui::End();

      // VL settings dialog
      if (showSettings) {
        ImGui::Begin("Volume Line(s) -> Settings");
        if (ImGui::InputFloat("min. importance", &minImportance, 0.025f, 1.0f, "%.4f")) {
          vl.setMinImportance(minImportance);
        }
        if (ImGui::SliderFloat("##P", &P, .001f, 2.f, "P: %.1f")) {
          vl.setP(P);
        }
        if (ImGui::Button("Toggle Bars/Lines")) {
          mode = mode==VolumeLines::Lines? VolumeLines::Bars: VolumeLines::Lines;
          vl.setMode(mode);
        }
        if (ImGui::Checkbox("Normalize", &normalize)) {
          vl.setNormalize(normalize);
        }
        if (ImGui::Checkbox("Use global color map", &globalColors)) {
          if (!globalColors) {
            vl.setGlobalColors(false, {});
            cmID = -1;
          }
        }
        if (globalColors) {
          qtOWL::ColorMapLibrary cmLib;
          std::vector<std::string> cmNames = cmLib.getNames();
          int currentID = cmID == -1 ? 0 : cmID;
          if (ImGui::BeginCombo("ColorMap", cmNames[currentID].c_str())) {
            for (int i=0; i<cmNames.size(); ++i) {
              bool isSelected = currentID == i;
              if (ImGui::Selectable(cmNames[i].c_str(), isSelected))
                currentID = i;
              if (isSelected)
                ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
          }

          if (currentID != cmID) {
            std::vector<vec4f> cm = cmLib.getMap(currentID).resampledTo(64);
            vl.setGlobalColors(true, cm);
            cmID = currentID;
          }
        }
        ImGui::End();
      }
    }

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    for (int f=0;f<numFields;++f) {
      if (tfe[f].cmapUpdated()) {
        vl.setColorMap(tfe[f].getColorMap(), f);
        renderer->setColorMap(tfe[f].getColorMap(), f);
        renderer->resetAccum();
      }

      if (tfe[f].rangeUpdated()){
        vl.setRange(tfe[f].getRange(), f);
        renderer->setRange(tfe[f].getRange(), f);
        renderer->setRelDomain(tfe[f].getRelDomain(), f);
        renderer->resetAccum();
      }

      if (tfe[f].opacityUpdated()){
        vl.setOpacityScale(tfe[f].getOpacityScale(), f);
        renderer->setOpacityScale(tfe[f].getOpacityScale(), f);
        renderer->resetAccum();
      }

      tfe[f].downdate();
    }

    // Set ROIs
    const bool roiEnabled = !vl.roisToHilbertIDs.empty();

    static auto previousROIs = vl.roisToHilbertIDs;
    static bool previousROIEnabled = roiEnabled;

    if (previousROIEnabled != roiEnabled){
      renderer->resetAccum();
      previousROIEnabled = roiEnabled;
      renderer->enableROI(roiEnabled, vl.cellBounds3D, 90.f);
    }

    // Set the ROIs' boxes
    if (previousROIs != vl.roisToHilbertIDs){
      renderer->resetAccum();
      previousROIs = vl.roisToHilbertIDs;

      for (int i=0; i<ROIS_MAX; ++i)
        renderer->setROI(i, {});

      for (int i=0; i < vl.roisToHilbertIDs.size(); ++i){
          if (i >= ROIS_MAX)
            break;
          renderer->setROI(i, vl.roisToHilbertIDs[i]);
      }
    }

//    if (tfe.updated()) {
//      vkt::LookupTable *lut = tfe.getUpdatedLookupTable();
//      if (lut != nullptr) {
//        auto dims = lut->getDims();
//        auto lutData = (float*)lut->getData();
//        float* colorVals = new float[dims.x*3];
//        float* alphaVals = new float[dims.x];
//        for (int i=0; i<dims.x; ++i) {
//          colorVals[i*3] = lutData[i*4];
//          colorVals[i*3+1] = lutData[i*4+1];
//          colorVals[i*3+2] = lutData[i*4+2];
//          alphaVals[i] = lutData[i*4+3];
//        }
//        // Apply transfer function
//        //updateLUT(colorVals,alphaVals,dims.x);
//        delete[] alphaVals;
//        delete[] colorVals;
//      }
//
//      // reset accum?!
//    }

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
        static int fieldID = 0;
        switch (fieldID) {
          case 0:
            cmdline.scalarFileName0 = argv[++i];
            break;
          case 1:
            cmdline.scalarFileName1 = argv[++i];
            break;
          case 2:
            cmdline.scalarFileName2 = argv[++i];
            break;
          case 3:
            cmdline.scalarFileName3 = argv[++i];
            break;
          case 4:
            cmdline.scalarFileName4 = argv[++i];
            break;
          case 5:
            cmdline.scalarFileName5 = argv[++i];
            break;
          case 6:
            cmdline.scalarFileName6 = argv[++i];
            break;
          case 7:
            cmdline.scalarFileName7 = argv[++i];
            break;
          default:
            throw std::runtime_error("too many fields");
        }
        fieldID++;
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
        static int xf = 0;
        cmdline.xfFileName[xf++] = argv[++i];
      }
      else if (arg == "--range") {
        cmdline.valueRange.lower = std::atof(argv[++i]);
        cmdline.valueRange.upper = std::atof(argv[++i]);
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
                         cmdline.scalarFileName0,
                         cmdline.scalarFileName1,
                         cmdline.scalarFileName2,
                         cmdline.scalarFileName3,
                         cmdline.scalarFileName4,
                         cmdline.scalarFileName5,
                         cmdline.scalarFileName6,
                         cmdline.scalarFileName7,
                         cmdline.kdtreeFileName,
                         cmdline.majorantsFileName,
                         cmdline.xform.remap_from,
                         cmdline.xform.remap_to,
                         cmdline.numMCs);

    const box3f modelBounds = renderer.modelBounds;

    // renderer.xf.colorMap = qtOWL::ColorMapLibrary().getMap(0);
    // renderer.setColorMap(renderer.xf.colorMap);

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

    Viewer viewer(&renderer);

    viewer.enableFlyMode();

    viewer.enableInspectMode(owl::viewer::OWLViewer::Arcball,modelBounds);
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

    if (auto mdl = renderer.model->as<ExaBrickModel>()) {
      for (int fieldID=0;fieldID<mdl->numFields;++fieldID) {
        if (!cmdline.xfFileName[fieldID].empty())
          viewer.tfe[fieldID].loadFromFile(cmdline.xfFileName[fieldID].c_str());
        viewer.tfe[fieldID].setRange(mdl->valueRanges[fieldID]);
      }
    }

    // Set up the volkit TFE
//    float rgba[] = {
//            1.f, 1.f, 1.f, .005f,
//            0.f, .1f, .1f, .25f,
//            .5f, .5f, .7f, .5f,
//            .7f, .7f, .07f, .75f,
//            1.f, .3f, .3f, 1.f
//            };
//    viewer.vktLUT = vkt::LookupTable(5,1,1,vkt::ColorFormat::RGBA32F);
//    viewer.vktLUT.setData((uint8_t*)rgba);
//    viewer.tfe.setLookupTableResource(viewer.vktLUT.getResourceHandle());
    // qtOWL::XFEditor *xfEditor = new qtOWL::XFEditor;
    // range1f valueRange = renderer.valueRange;
    // if (!cmdline.valueRange.is_empty())
    //   valueRange = cmdline.valueRange;

    // QObject::connect(xfEditor,&qtOWL::XFEditor::colorMapChanged,
    //                  &viewer, &Viewer::colorMapChanged);
    // QObject::connect(xfEditor,&qtOWL::XFEditor::rangeChanged,
    //                  &viewer, &Viewer::rangeChanged);
    // QObject::connect(xfEditor,&qtOWL::XFEditor::opacityScaleChanged,
    //                  &viewer, &Viewer::opacityScaleChanged);
    // QObject::connect(&viewer.lightInteractor,&LightInteractor::lightPosChanged,
    //                  &viewer, &Viewer::lightPosChanged);

    // viewer.xfEditor = xfEditor;

    // if (cmdline.xfFileName != "")
    //   xfEditor->loadFrom(cmdline.xfFileName);
    // xfEditor->setAbsDomain(valueRange);
    // viewer.opacityScaleChanged(xfEditor->getOpacityScale());

    viewer.showAndRun();
  }
} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

