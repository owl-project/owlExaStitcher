BUILD_DIR="/home/zellmann/owlExaStitcher/build"
UMESH="/slow/stitcher/landinggear.umesh"
GRIDLETS="-grids /slow/stitcher/landinggear.grids"
BRICKS="-bricks /mnt/raid/zellmann/ExaBrick/landinggear/chombo.exa.bricks"
#BRICKS="-bricks /home/zellmann/exabuilder/build/gear.morton.bricks"
SCALARS="-scalars /mnt/raid/zellmann/exa_from_trinity/landinggear/chombo.exa.velmag.scalars"
KDTREE="-kdtree /slow/stefan/gear.kd"
MESH="-mesh /mnt/raid/zellmann/ExaBrick/landinggear/landingGearThickFilledGap.tris"
CAMERA="--camera -0.1688781381 0.7803725004 0.2884992957 19.29099655 -20.87918472 -2.018322945 0 0 -1 --fov 40"
XFORM="--remap-from 0 0 0 131071 131071 65535 --remap-to -16 -16 -.1 16 16 16"
#LIGHT="--light -1.811209 1.815962 0.834571 0.0000120"
IMG_SIZE="-win 1024 1024"
NUM_MCS="--num-mcs 1024 1024 256"
XF="-xf gear.xf"

fpsfile="-fps gear.fps"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF
cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

##outfile="-o gear_stitcher"
##${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee gear.out


#--- ExaBricks benchmark ----------------------------------
samplers=("ABRs" "Extended bricks")
iterators=("ABRs" "DDA" "MC-BVH" "KDTree" "Brick-BVH")

for sampler_mode in {1..1}
do
  for traversal_mode in {5..5}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_MIRROR_EXAJET=OFF \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}"

    outfile="-o gear_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee gear.out
  done
done
