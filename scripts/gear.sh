BUILD_DIR="/home/ampere/stefan/exastitch/build"
BRICKS="-bricks /slow/stefan/gear.bricks"
SCALARS="-scalars /fast/exa/landinggear/derived/chombo.exa.velmag.scalars"
KDTREE="-kdtree /slow/stefan/gear.kd"
MESH="-mesh /fast/exa/landinggear/landingGearThickFilledGap.tris"
CAMERA="--camera -0.1688781381 0.7803725004 0.2884992957 19.29099655 -20.87918472 -2.018322945 0 0 -1 --fov 40"
XFORM="--remap-from 0 0 0 131071 131071 65535 --remap-to -16 -16 -.1 16 16 16"
IMG_SIZE="-win 1024 1024"
NUM_MCS="--num-mcs 1024 1024 256"
XF="-xf gear.xf"

for sampler_mode in {0..1}
do
  for traversal_mode in {0..4}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=OFF \
        -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=ON \
        -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=OFF \
        -DEXA_STITCH_MIRROR_EXAJET=OFF \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    outfile="-o gear_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} -rt 0 2>&1 | tee gear.out
  done
done