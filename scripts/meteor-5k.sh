BUILD_DIR="/home/zellmann/owlExaStitcher/build"
#BRICKS="-bricks /home/zellmann/tmp/meteor-5700.morton.bricks"
#BRICKS="-bricks /home/zellmann/tmp/meteor-5700.exa.projection.bricks"
BRICKS="-bricks /home/zellmann/tmp/meteor-5700.exa.sah.bricks"
#BRICKS="-bricks /home/zellmann/exabuilder/build/meteor-5700.projection.bricks"
SCALARS="-scalars /mnt/raid/zellmann/exa/meteor-5k/meteor-5700.exa.tev.scalars"
#KDTREE="-kdtree /slow/stefan/meteor-20060.kd"

CAMERA="--camera 2607.41 3218.14 3474.3 494.535 1300.13 -210.744 -0.242807 0.91051 -0.334689 -fovy 70"
#broken:
CAMERA="--camera -1930.1 2174.56 704.562 -1930.1 2174.56 704.561 0 1 0 -fovy 70"
XF="-xf meteor-5k.xf"
#LIGHT="--light -5166.112793 5752.800293 4556.298340 100.000000"
CLIP_PLANE="--clip-plane 0 0 1 921.6"
NUM_MCS="--num-mcs 128 128 64"
IMG_SIZE="-win 4096 4096"

fpsfile="-fps meteor-5k.fps"

#--- Stitcher benchmark -----------------------------------
#cmake ${BUILD_DIR} \
#    -DEXA_STITCH_MIRROR_EXAJET=OFF
#cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

#outfile="-o meteor-5k_stitcher"
#${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-5k.out


#--- ExaBricks benchmark ----------------------------------
samplers=("ABRs" "Extended bricks")
iterators=("ABRs" "DDA" "MC-BVH" "KDTree" "Brick-BVH")

for sampler_mode in {0..0}
do
  for traversal_mode in {0..0}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_MIRROR_EXAJET=OFF \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    #echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}"

    outfile="-o meteor-5k_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee meteor-5k.out
  done
done
