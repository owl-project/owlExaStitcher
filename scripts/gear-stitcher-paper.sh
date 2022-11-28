BUILD_DIR="/home/ampere/stefan/exastitch/build"
UMESH="/slow/stitcher/landinggear.umesh"
GRIDLETS="-grids /slow/stitcher/landinggear.grids"
BM="-bm /slow/stitcher/landinggear_full.bm"
BRICKS="-bricks /slow/stefan/gear.bricks"
SCALARS="-scalars /fast/exa/landinggear/derived/chombo.exa.velmag.scalars"
KDTREE="-kdtree /slow/stefan/gear.kd"
MESH="-mesh /fast/exa/landinggear/landingGearThickFilledGap.tris"
CAMERA="--camera -0.1688781381 0.7803725004 0.2884992957 19.29099655 -20.87918472 -2.018322945 0 0 -1 --fov 40"
XFORM="--remap-from 0 0 0 131071 131071 65535 --remap-to -16 -16 -.1 16 16 16"
#LIGHT="--light -1.811209 1.815962 0.834571 0.0000120"
IMG_SIZE="-win 1024 1024"
NUM_MCS="--num-mcs 1024 1024 256"
XF="-xf gear-stitcher-paper.xf"

fpsfile="-fps gear.fps"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF \
    -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=0 \
    -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=1
cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

outfile="-o gear_stitcher"
#${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee gear.out

echo "Benchmark: Bigmesh"

outfile="-o gear_bigmesh"
${BUILD_DIR}/exaStitchHeadlessViewer ${BM} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee gear.out

echo "Benchmark: Exabrick"

outfile="-o gear_exabrick"
${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee gear.out
