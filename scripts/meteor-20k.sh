BUILD_DIR="/home/ampere/stefan/exastitch/build"
BRICKS="-bricks /slow/stefan/meteor-20060.bricks"
SCALARS="-scalars /slow/UNSORTED/exa/meteor-20k/meteor-20060.exa.tev.scalars"
KDTREE="-kdtree /slow/stefan/meteor-20060.kd"

CAMERA="--camera 2607.41 3218.14 3474.3 494.535 1300.13 -210.744 -0.242807 0.91051 -0.334689 -fovy 70"
XF="-xf meteor-20k.xf"
#LIGHT="--light -5166.112793 5752.800293 4556.298340 100.000000"
CLIP_PLANE="--clip-plane 0 0 1 921.6"
NUM_MCS="--num-mcs 128 128 64"
IMG_SIZE="-win 1024 1024"

fpsfile="-fps meteor-20k.fps"

samplers=("ABRs" "Extended bricks")
iterators=("ABRs" "DDA" "MC-BVH" "KDTree" "Brick-BVH")

for sampler_mode in {0..1}
do
  for traversal_mode in {0..4}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_MIRROR_EXAJET=OFF \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}"

    outfile="-o meteor-20k_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-20k.out
  done
done
