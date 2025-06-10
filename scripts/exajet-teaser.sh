BUILD_DIR="/home/zellmann/owlExaStitcher/build"
#BRICKS="-bricks /home/zellmann/exabuilder/build/out.projection.bricks"
#BRICKS="-bricks /home/zellmann/exabuilder/build/out.morton.bricks"
BRICKS="-bricks /mnt/raid/zellmann/exa/exajet/exajet-vah.bricks"
SCALARS="-scalars /mnt/raid/zellmann/mag_vorticity.bin"
MESH="-mesh /mnt/raid/zellmann/exajet.exa.tris"

CAMERA="--camera -9.5868082047 -4.4676966667 -1.4105665684 2.2676076889 -1.6570863724 -0.4713226557 0.0000000000 0.0000000000 1.0000000000 --fov 10"
XF="-xf exajet-teaser.xf"
NUM_MCS="--num-mcs 512 512 512"
XFORM="--remap-from 1232128 1259072 1238336 1270848 1277952 1255296 --remap-to -1.73575 -9.44 -3.73281 17.6243 0 4.74719"
IMG_SIZE="--size 2500 625"

fpsfile="-fps exajet-teaser.fps"

for sampler_mode in {1..1}
do
  for traversal_mode in {5..5}
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
    #gdb --args ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet-teaser.out
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${XFORM} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet-teaser.out
  done
done
