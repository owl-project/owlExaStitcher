BUILD_DIR="/home/ampere/stefan/exastitch/build"
UMESH="/slow/stitcher/meteor-46k.umesh"
GRIDLETS="-grids /slow/stitcher/meteor-46k.grids.8"
BRICKS="-bricks /slow/qiwu/meteor-46112.bricks"
SCALARS="-scalars /slow/UNSORTED/exa/meteor-46k/meteor-46112.tev.scalars"
BM="-bm /slow/stitcher/meteor-46k_full.bm "

#CAMERA=""
XF="-xf meteor-46k-stitcher-paper.xf"
NUM_MCS="--num-mcs 128 128 64"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF \
    -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=0 \
    -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=1
cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

outfile="-o meteor-46k_stitcher"
${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-46k.out

echo "Benchmark: Bigmesh"

outfile="-o meteor-46k_bigmesh"
${BUILD_DIR}/exaStitchHeadlessViewer ${BM} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} $SUBIMG ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-46k.out

echo "Benchmark: Exabrick"

outfile="-o meteor-46k_exabrick"
${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-46k.out
