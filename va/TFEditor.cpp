#include "TFEditor.h"

#include <imgui.h>

namespace exa{
  using namespace owl;

  TFEditor::TFEditor(){
    float rgba[] = {
        1.f, 1.f, 1.f, .005f,
        0.f, .1f, .1f, .25f,
        .5f, .5f, .7f, .5f,
        .7f, .7f, .07f, .75f,
        1.f, .3f, .3f, 1.f
    };
    vktLUT = vkt::LookupTable(5,1,1,vkt::ColorFormat::RGBA32F);
    vktLUT.setData((uint8_t*)rgba);
    vktTFE.setLookupTableResource(vktLUT.getResourceHandle());

    range = {0.f, 1.f};
    relDomain = {0.f, 100.f};
    opacityScale = 100.f;

    firstFrame = true;
    cmapUpdated_ = true;
    opacityUpdated_ = true;
    rangeUpdated_ = true;
  }

  void TFEditor::drawImmediate() {
    if (!firstFrame) {
      cmapUpdated_ = false;
      opacityUpdated_ = false;
      rangeUpdated_ = false;
    } else {
      firstFrame = false;
    }

    vktTFE.drawImmediate();

    if (vktTFE.updated())
      cmapUpdated_ = true;

    interval<float> curRange = range;
    ImGui::DragFloatRange2("Range", &curRange.lo, &curRange.hi,
                           max(curRange.diagonal()/100.f, 0.0001f));

    if (curRange != range) {
      rangeUpdated_ = true;
      range.lo = std::min(curRange.lo, curRange.hi);
      range.hi = std::max(curRange.lo, curRange.hi);
    }

    interval<float> curRelDomain = relDomain;
    ImGui::DragFloatRange2("Rel Domain", &curRelDomain.lo, &curRelDomain.hi, 1.f);

    if (curRelDomain != relDomain) {
      rangeUpdated_ = true;
      relDomain.lo = std::min(curRelDomain.lo, curRelDomain.hi);
      relDomain.hi = std::max(curRelDomain.lo, curRelDomain.hi);
    }

    float curOpacityScale = opacityScale;
    ImGui::DragFloat("Opacity", &curOpacityScale, 1.f);

    if (curOpacityScale != opacityScale) {
      opacityUpdated_ = true;
      opacityScale = curOpacityScale;
    }
  }

  std::vector<vec4f> TFEditor::getColorMap(){
    vkt::LookupTable *lut = vktTFE.getUpdatedLookupTable();
    if (lut != nullptr) {
      auto dims = lut->getDims();
      auto lutData = (float *)lut->getData();

      std::vector<vec4f> cmap;
      cmap.resize(dims.x);
      std::copy(&lutData[0], &lutData[dims.x*4], (float*)&cmap[0]);
      return cmap;
    }

    return {};
}
}