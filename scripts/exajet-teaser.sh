BUILD_DIR="/home/ampere/stefan/exastitch/build"
BRICKS="-bricks /slow/UNSORTED/exa/exajet/exajet-spatial-32min.bricks"
SCALARS="-scalars /slow/UNSORTED/exa/exajet/mag_vorticity.bin"
KDTREE="-kdtree /slow/UNSORTED/exa/exajet/exajet-spatial-32min.kdtree"
MESH="-mesh /slow/UNSORTED/exa/exajet/exajet.exa.tris"

CAMERA="--camera -9.5868082047 -4.4676966667 -1.4105665684 2.2676076889 -1.6570863724 -0.4713226557 0.0000000000 0.0000000000 1.0000000000 --fov 10"
XF="-xf exajet-teaser.xf"
NUM_MCS="--num-mcs 512 512 512"
XFORM="--remap-from 1232128 1259072 1238336 1270848 1277952 1255296 --remap-to -1.73575 -9.44 -3.73281 17.6243 0 4.74719"
IMG_SIZE="--size 2500 625"

for sampler_mode in {0..1}
do
  for traversal_mode in {0..4}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=OFF \
        -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=ON \
        -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=OFF \
        -DEXA_STITCH_MIRROR_EXAJET=ON \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    outfile="-o exajet-teaser_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MS} ${XFORM} ${IMG_SIZE} ${outfile} -rt 0 2>&1 | tee exajet-teaser.out
  done
done
