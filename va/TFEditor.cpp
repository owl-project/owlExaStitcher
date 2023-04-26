#include "TFEditor.h"

#include <fstream>
#include <imgui.h>

#include <qtOWL/ColorMaps.h>


namespace exa{
  using namespace owl;

  TFEditor::TFEditor(){
    cmapNames = cmapLib.getNames();
    selectedCMapID = -1;
    selectCMap(0, false);

    range = {0.f, 1.f};
    relDomain = {0.f, 100.f};
    opacityScale = 100.f;

    firstFrame = true;
  }

  void TFEditor::drawImmediate() {
    cmapUpdated_ = firstFrame;
    opacityUpdated_ = firstFrame;
    rangeUpdated_ = firstFrame;

    if (firstFrame)
      firstFrame = false;

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

    int curCMapID = selectedCMapID;

    if (ImGui::BeginCombo("CMap", cmapNames[curCMapID].c_str()))
    {
      for (int i = 0; i < cmapNames.size(); ++i)
      {
        bool isSelected = curCMapID == i;
        if (ImGui::Selectable(cmapNames[i].c_str(), isSelected))
            curCMapID = i;
        if (isSelected)
            ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();

      selectCMap(curCMapID, true);
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


  void TFEditor::loadFromFile(const char *fname) {
    std::ifstream xfFile(fname, std::ios::binary);

    if (!xfFile.good())
      throw std::runtime_error("Could not open TF");

    static const size_t xfFileFormatMagic = 0x1235abc000;
    size_t magic;
    xfFile.read((char*)&magic,sizeof(xfFileFormatMagic));
    if (magic != xfFileFormatMagic) {
      throw std::runtime_error("Not a valid TF file");
    }

    xfFile.read((char*)&opacityScale,sizeof(opacityScale));

    xfFile.read((char*)&range.lower,sizeof(range.lower));
    xfFile.read((char*)&range.upper,sizeof(range.upper));

    xfFile.read((char*)&relDomain,sizeof(relDomain));

    std::vector<float> colorMap;
    int numColorMapValues;
    xfFile.read((char*)&numColorMapValues,sizeof(numColorMapValues));
    colorMap.resize(numColorMapValues*4);
    xfFile.read((char*)colorMap.data(),colorMap.size()*sizeof(colorMap[0]));

    vktLUT = vkt::LookupTable(numColorMapValues,1,1,vkt::ColorFormat::RGBA32F);
    vktLUT.setData((uint8_t*)colorMap.data());
    vktTFE.setLookupTableResource(vktLUT.getResourceHandle());

    firstFrame = true;
  }

  void TFEditor::selectCMap(int cmapID, bool keepAlpha){
    if (cmapID == selectedCMapID)
      return;

    selectedCMapID = cmapID;

    auto map = cmapLib.getMap(cmapID);//.resampledTo(numColors);

    vktLUT = vkt::LookupTable(map.size(),1,1,vkt::ColorFormat::RGBA32F);
    vktLUT.setData((uint8_t*)map.data());
    vktTFE.setLookupTableResource(vktLUT.getResourceHandle());

    auto updatedLUT = vktTFE.getUpdatedLookupTable();
    if (updatedLUT != nullptr){
      auto rmap = map.resampledTo(updatedLUT->getDims().x);

      if (keepAlpha){
        auto currentCMap = getColorMap();

        if (rmap.size() != currentCMap.size())
            throw std::runtime_error("TFE error");

        for (int i=0; i<rmap.size(); ++i)
          rmap[i].w = currentCMap[i].w;
      }

      updatedLUT->setData((uint8_t*)&rmap[0]);
    }

    cmapUpdated_ = true;
  }
}
