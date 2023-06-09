BUILD_DIR="/home/ampere/stefan/exastitch/build"
UMESH="/slow/stitcher/exajet.umesh"
GRIDLETS="-grids /slow/stitcher/exajet.grids"
BRICKS="-bricks /slow/UNSORTED/exa/exajet/exajet-spatial-32min.bricks"
SCALARS="-scalars /slow/UNSORTED/exa/exajet/mag_vorticity.bin"
MESH="-mesh /slow/UNSORTED/exa/exajet/exajet.exa.tris"
BM="-bm /slow2/exa-bm/n1/exajet.bm"
#CAMERA="--camera 18.3118 -0.61511 0.666788 9.86533 -6.74496 -3.40858 -0.434744 0.000316143 0.900555 -fovy 70"
CAMERA="--camera 14.3317 -5.60984 4.24107 10.0979 -0.428769 -2.45186 -0.325708 0.636723 0.698925 -fovy 70"
XF="-xf exajet-stitcher-paper.xf"
XFORM="--remap-from 1232128 1259072 1238336 1270848 1277952 1255296 --remap-to -1.73575 -9.44 -3.73281 17.6243 0 4.74719"
IMG_SIZE="--size 1024 1024"
NUM_MCS="--num-mcs 512 512 512"

fpsfile="-fps exajet.fps"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF \
    -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=0 \
    -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=1
cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

outfile="-o exajet_stitcher"
${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${XFORM} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet.out

echo "Benchmark: Bigmesh"

outfile="-o exajet_bigmesh"
${BUILD_DIR}/exaStitchHeadlessViewer ${BM} ${MESH} ${CAMERA} ${XF} ${XFORM} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet.out

echo "Benchmark: Exabrick"

outfile="-o exajet_exabrick"
${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${XFORM} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet.out
