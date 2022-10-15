BUILD_DIR="/home/ampere/stefan/exastitch/build"
BRICKS="-bricks /slow/UNSORTED/exa/exajet/exajet-spatial-32min.bricks"
SCALARS="-scalars /slow/UNSORTED/exa/exajet/mag_vorticity.bin"
KDTREE="-kdtree /slow/UNSORTED/exa/exajet/exajet-spatial-32min.kdtree"
MESH="-mesh /slow/UNSORTED/exa/exajet/exajet.exa.tris"

CAMERA="--camera 15.48 0 6.675 8.677 0 -3.46 0 0 1 --fov 70"
XF="-xf exajet-teaser.xf"
#LIGHT="--light 17.6243 9.44 4.74719 0.0004"
NUM_MCS="--num-mcs 512 512 512"
XFORM="--remap-from 1232128 1259072 1238336 1270848 1277952 1255296 --remap-to -1.73575 -9.44 -3.73281 17.6243 0 4.74719"
IMG_SIZE="--size 1024 1024"

fpsfile="-fps exajet-rear.fps"

samplers=("ABRs" "Extended bricks")
iterators=("ABRs" "DDA" "MC-BVH" "KDTree" "Brick-BVH")

for sampler_mode in {0..1}
do
  for traversal_mode in {0..4}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_MIRROR_EXAJET=ON \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}"

    outfile="-o exajet-rear_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${XFORM} ${IMG_SIZE} ${LIGHT} ${outfile} ${fpsfile} -rt 0 2>&1 | tee exajet-rear.out
  done
done
