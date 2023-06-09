Some code to illustrate how to use the device:
```
anariVolume = anariNewVolume(device,"scivis");

anariSpatialField = anariNewSpatialField(device,"amr");

std::vector<visionaray::aabbi> blockBoundData(2);
// see here: https://github.com/openvkl/openvkl/blob/master/openvkl/devices/cpu/volume/amr/AMRData.h#L28
// how these are defined (upper is an integer coordinate and does not include the
// extent of the rightmost cells)
blockBoundData[0] = {
    {0,0,0},{1,1,1} // thats a 2^3 box
};
blockBoundData[1] = {
    {1,0,0},{1,0,0}
};
ANARIArray1D blockBounds = anariNewArray1D(device,blockBoundData.data(),0,0,
                                           ANARI_INT32_BOX3,blockBoundData.size(),
                                           0);

std::vector<int> levelData(2);
levelData[0] = 0;
levelData[1] = 1;
ANARIArray1D blockLevel = anariNewArray1D(device,levelData.data(),0,0,
                                          ANARI_INT32,levelData.size(),0);

std::vector<ANARIArray3D> blockDataData(2);
std::vector<float> block0(8); for (int i=0; i<8; ++i) block0[i] = i/float(8);
std::vector<float> block1(1); block1[0] = 1.f;
blockDataData[0] = anariNewArray3D(device,block0.data(),0,0,ANARI_FLOAT32,
                                   2,2,2,0,0,0);
blockDataData[1] = anariNewArray3D(device,block1.data(),0,0,ANARI_FLOAT32,
                                   1,1,1,0,0,0);
ANARIArray1D blockData = anariNewArray1D(device,blockDataData.data(),0,0,
                                         ANARI_ARRAY3D,blockDataData.size(),0);

anariSetParameter(device,anariSpatialField,"block.bounds",ANARI_ARRAY1D,
                  &blockBounds);
anariSetParameter(device,anariSpatialField,"block.level",ANARI_ARRAY1D,
                  &blockLevel);
anariSetParameter(device,anariSpatialField,"block.data",ANARI_ARRAY1D,&blockData);
anariCommitParameters(device,anariSpatialField);

anariSetParameter(device,anariVolume,"field",ANARI_SPATIAL_FIELD,
                  &anariSpatialField);

rgbLUT.resize(numEntries*3);
alphaLUT.resize(numEntries);
std::copy(rgb,rgb+numEntries*3,rgbLUT.data());
std::copy(alpha,alpha+numEntries,alphaLUT.data());

ANARIArray1D anariColor = anariNewArray1D(device, rgbLUT.data(), 0, 0,
                                          ANARI_FLOAT32_VEC3, rgbLUT.size(), 0);
ANARIArray1D anariOpacity = anariNewArray1D(device, alphaLUT.data(), 0, 0,
                                            ANARI_FLOAT32, alphaLUT.size(), 0);

anariSetParameter(device, anariVolume, "color", ANARI_ARRAY1D, &anariColor);
anariSetParameter(device, anariVolume, "opacity", ANARI_ARRAY1D, &anariOpacity);

anariCommitParameters(device,anariVolume);

anariRelease(device, anariColor);
anariRelease(device, anariOpacity);
anariCommitParameters(device,anariVolume);

anariRelease(device,blockBounds);
anariRelease(device,blockLevel);
anariRelease(device,blockDataData[0]);
anariRelease(device,blockDataData[1]);
anariRelease(device,blockData);
```
