BUILD_DIR="/home/ampere/stefan/exastitch/build"
UMESH="/slow/stitcher/cloud.umesh"
UMESH_FULL="/slow/stitcher/cloud_full.umesh"
SCALARS="-scalars /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.scalars"
GRIDLETS="-grids /slow/stitcher/cloud.grids.8"
BM="-bm /slow/stitcher/cloud_full.bm"
BRICKS="-bricks /slow/qiwu/SILCC_hdf5_plt_cnt_0300_dens.bricks"
CAMERA="--camera 5436.36 3320.92 82010 6671.5 2211.75 81920.5 -0.655946 -0.709695 -0.257033 -fovy 70"
XF="-xf cloud-stitcher-paper.xf"
NUM_MCS="--num-mcs 128 128 128"
IMG_SIZE="-win 1024 1024"

fpsfile="-fps cloud.fps"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF \
    -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=0 \
    -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=1
cmake --build ${BUILD_DIR} -j

echo "Benchmark: UMesh Full"

outfile="-o cloud_umesh-full"
${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH_FULL} ${SCALARS} ${CAMERA} ${XF} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out

echo "Benchmark: Bigmesh"

outfile="-o cloud_bigmesh"
${BUILD_DIR}/exaStitchHeadlessViewer ${BM} ${CAMERA} ${XF} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out

echo "Benchmark: Exabrick"

outfile="-o cloud_exabrick"
${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${CAMERA} ${XF} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out

echo "Benchmark: Stitcher"

outfile="-o cloud_stitcher"
${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${CAMERA} ${XF} ${NUM_MCS} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out

