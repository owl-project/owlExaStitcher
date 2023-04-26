#pragma once

#include <vector>
#include "owl/common.h"
#include "volkit/src/vkt/TransfuncEditor.hpp"
#include <qtOWL/ColorMaps.h>

namespace exa{
  class TFEditor
  {
  public:
    TFEditor();

    void drawImmediate();

    bool cmapUpdated() const { return cmapUpdated_; }
    bool rangeUpdated() const { return rangeUpdated_; }
    bool opacityUpdated() const { return opacityUpdated_; }

    void downdate() { cmapUpdated_ = rangeUpdated_ = opacityUpdated_ = false; }

    std::vector<owl::vec4f> getColorMap();
    owl::interval<float> getRange() const { return range; }
    owl::interval<float> getRelDomain() const { return relDomain; }
    float getOpacityScale() const {return opacityScale;}
    void setRange(owl::interval<float> r) { range = r; }

    void loadFromFile(const char* fname);
    void saveToFile(const char* fname);

    std::string saveFilename = "owlDVR.xf";
  private:
    vkt::LookupTable vktLUT;
    vkt::TransfuncEditor vktTFE;

    bool firstFrame;
    bool cmapUpdated_;
    bool rangeUpdated_;
    bool opacityUpdated_;

    owl::interval<float> range;
    owl::interval<float> relDomain;
    float opacityScale;

    qtOWL::ColorMapLibrary cmapLib;
    std::vector<std::string> cmapNames;

    int selectedCMapID;

    void selectCMap(int cmapID, bool keepAlpha);
  };

}
