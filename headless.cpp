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

#include <fstream>
#include <iomanip>
#include <QApplication>
#include <QDesktopWidget>
#include <cuda_runtime.h>
#include "headless.h"

// #define STB_IMAGE_WRITE_IMPLEMENTATION 1
#include "stb/stb_image_write.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

using namespace owl;

namespace exa {

  static std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
#ifdef _WIN32
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd, "r"), _pclose);
#else
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
#endif
    if (!pipe) {
      throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    return result;
  }

  Headless::Headless(const std::string &title, const vec2i &initWindowSize)
    : fbSize(initWindowSize)
    , title(title)
  {
    owl::vec2i ws = initWindowSize;
    if (ws == vec2i(0,0)) {
      QRect rec = QApplication::desktop()->screenGeometry();
      int height = rec.height();
      int width = rec.width();
      ws = vec2i{width,height};
    }

    cudaMalloc(&fbPointer,sizeof(uint32_t)*ws.x*ws.y);
    fbSize = ws;
  }

  Headless::~Headless()
  {
    cudaFree(fbPointer);
  }

  void Headless::setOutFileName(std::string fileName)
  {
    outFileName = fileName;
  }

  void Headless::setOutFileNameFPS(std::string fileName)
  {
    outFileNameFPS = fileName;
  }

  struct Log {

    std::string str() { return stream.str(); }
    std::stringstream stream;
  };

  template <typename T>
  Log& operator<<(Log &log, T&& t)
  {
    log.stream << t;
    std::cout << t << std::flush;
    return log;
  }

  void Headless::run()
  {
    Log log;

    log << "Benchmark summary:\n";
    log << "==================\n\n";

    int frameID=0;
    int screenshotID=10;//-1
    int stopID=510;
    std::string screenshotFileName = outFileName.empty() ? "" : outFileName+".png";

    log << '\n';
    log << exec("nvidia-smi");

    log << "\nBechmark:\n";
    log << "FRAME_ID;SEC.\n";

    double totalTime = 0.0;
    double totalNumFrames = 0;

    while (++frameID) {

      static double lastCameraUpdate = -1.f;
      if (camera.lastModified != lastCameraUpdate) {
        cameraChanged();
        lastCameraUpdate = camera.lastModified;
      }

      double t1 = getCurrentTime();
      render();
      double t2 = getCurrentTime();
      log << frameID << ';' << t2-t1 << '\n';

      if (frameID==screenshotID) {
        // Save png
        std::string fileName = screenshotFileName.empty() ? "offline.png" : screenshotFileName;
        std::vector<uint32_t> pixels(fbSize.x*fbSize.y);
        cudaError_t err = cudaMemcpy(pixels.data(),fbPointer,fbSize.x*fbSize.y*sizeof(uint32_t),
                                     cudaMemcpyDeviceToHost);

        if (err==cudaSuccess) {
          //stbi_flip_vertically_on_write(1);
          std::vector<uint32_t> pixelsFlipped;
          for (int y=0;y<fbSize.y;y++) {
            const uint32_t *line = pixels.data() + (fbSize.y-1-y)*fbSize.x;
            for (int x=0;x<fbSize.x;x++) {
              pixelsFlipped.push_back(line[x] | (0xff << 24));
            }
          }
          stbi_write_png(fileName.c_str(),fbSize.x,fbSize.y,4,
                         pixelsFlipped.data(),fbSize.x*sizeof(uint32_t));
          std::cout << "#owl.viewer: frame buffer written to " << fileName << std::endl;
        } else {
          std::cerr << cudaGetErrorString(cudaGetLastError()) << '\n';
        }
      }

      if (frameID>screenshotID) {
        totalTime += t2-t1;
        totalNumFrames++;
      }

      if (frameID==stopID)
        break;
    }

    double FPS = 1./(totalTime/totalNumFrames);
    log << (totalTime/totalNumFrames) << " avg. secs over " << totalNumFrames << " time\n";
    log << FPS << " FPS\n";

    std::string logFileName = outFileName.empty() ? "benchmark.log" : outFileName+".log";
    std::ofstream logFile(logFileName);
    logFile << log.str();

    auto now = []() {
      auto nnow = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(nnow);

      std::stringstream ss;
      ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
      return ss.str();
    };

    std::string fpsFileName = outFileName.empty() ? "fps.log" : outFileNameFPS;
    std::ofstream fpsFile(fpsFileName,std::ios_base::app);
    fpsFile << now() << ',' << outFileName << ',' << FPS << '\n';
  }

  void Headless::resize(const owl::vec2i &newSize)
  {
    fbSize = newSize;

    cudaFree(fbPointer);
    cudaMalloc(&fbPointer,fbSize.x*fbSize.y*sizeof(uint32_t));

    setAspect(newSize.x/(float)newSize.y);
  }

  void Headless::setTitle(const std::string &s)
  {
    title = s;
    // std::cout << title << '\n';
  }

  /*! helper function that dumps the current frame buffer in a png
    file of given name */
  void Headless::screenShot(const std::string &fileName)
  {
    const uint32_t *fb
      = (const uint32_t*)fbPointer;

    std::vector<uint32_t> pixels;
    for (int y=0;y<fbSize.y;y++) {
      const uint32_t *line = fb + (fbSize.y-1-y)*fbSize.x;
      for (int x=0;x<fbSize.x;x++) {
        pixels.push_back(line[x] | (0xff << 24));
      }
    }
    stbi_write_png(fileName.c_str(),fbSize.x,fbSize.y,4,
                   pixels.data(),fbSize.x*sizeof(uint32_t));
    std::cout << "#owl.viewer: frame buffer written to " << fileName << std::endl;
  }

  void Headless::setWorldScale(const float worldScale)
  {
    camera.motionSpeed = worldScale / sqrtf(3.f);
  }

    /*! re-computes the 'camera' from the 'cameracontrol', and notify
    app that the camera got changed */
  void Headless::updateCamera()
  {
    // camera.digestInto(simpleCamera);
    // if (isActive)
    camera.lastModified = getCurrentTime();
  }

    /*! set a new orientation for the camera, update the camera, and
    notify the app */
  void Headless::setCameraOrientation(/* camera origin    : */const vec3f &origin,
                                      /* point of interest: */const vec3f &interest,
                                      /* up-vector        : */const vec3f &up,
                                      /* fovy, in degrees : */float fovyInDegrees)
  {
    camera.setOrientation(origin,interest,up,fovyInDegrees,false);
    updateCamera();
  }

  inline owl::interval<float> order(const owl::interval<float> v)
  {
    return { std::min(v.lower,v.upper),std::max(v.lower,v.upper) };
  }

  inline float lerp(const owl::interval<float> v, const float f)
  {
    return (1.f-f)*v.lower + f*v.upper;
  }

  static const size_t xfFileFormatMagic = 0x1235abc000;
  void Headless::loadTransferFunction(const std::string &fileName)
  {
    std::ifstream in(fileName,std::ios::binary);
    size_t magic;
    in.read((char*)&magic,sizeof(xfFileFormatMagic));
    if (magic != xfFileFormatMagic) {
      throw std::runtime_error(fileName+": not a valid '.xf' "
                               "transfer function file!?");
    } else {
      in.read((char*)&xf.opacityScale,sizeof(xf.opacityScale));

      in.read((char*)&xf.absDomain.lower,sizeof(xf.absDomain.lower));
      in.read((char*)&xf.absDomain.upper,sizeof(xf.absDomain.upper));

      in.read((char*)&xf.relDomain.lower,sizeof(xf.relDomain.lower));
      in.read((char*)&xf.relDomain.upper,sizeof(xf.relDomain.upper));

      int numColorMapValues;
      in.read((char*)&numColorMapValues,sizeof(numColorMapValues));
      xf.colorMap.resize(numColorMapValues);
      in.read((char*)xf.colorMap.data(),xf.colorMap.size()*sizeof(xf.colorMap[0]));
    }

    std::cout << "loaded xf from " << fileName << std::endl;
  }

  owl::interval<float> Headless::xfRange()
  {
    owl::interval<float> absRange = xf.absDomain;
    absRange = order(absRange);

    owl::interval<float> relRange(.01f*xf.relDomain.lower,
                                  .01f*xf.relDomain.upper);
    relRange = order(relRange);

    owl::interval<float> finalRange(lerp(absRange,relRange.lower),
                                    lerp(absRange,relRange.upper));

    return finalRange;
  }

} // ::exa

// vim: sw=2:expandtab:softtabstop=2:ts=2:cino=\:0g0t0

