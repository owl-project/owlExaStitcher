BUILD_DIR="/home/ampere/stefan/exastitch/build"
UMESH="/slow/stitcher/cloud.umesh"
GRIDLETS="-grids /slow/stitcher/cloud.grids.8"
BRICKS="-bricks /slow/qiwu/SILCC_hdf5_plt_cnt_0300_dens.bricks"
SCALARS="-scalars /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.scalars"
KDTREE="-kdtree /slow/qiwu/SILCC_hdf5_plt_cnt_0300_dens.kd"

CAMERA="--camera 5346.1 1520.14 82012.8 6671.5 2211.75 81920.5 -0.430131 0.759096 -0.488631 -fovy 70"
XF="-xf cloud.xf"
#LIGHT="--light -13885.075195 3776.682129 86940.859375 900.000000"
NUM_MCS="--num-mcs 128 128 128"
IMG_SIZE="-win 1024 1024"

fpsfile="-fps cloud.fps"

#--- Stitcher benchmark -----------------------------------
cmake ${BUILD_DIR} \
    -DEXA_STITCH_MIRROR_EXAJET=OFF
cmake --build ${BUILD_DIR} -j

echo "Benchmark: Stitcher"

outfile="-o cloud_stitcher"
${BUILD_DIR}/exaStitchHeadlessViewer ${UMESH} ${GRIDLETS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out


#--- ExaBricks benchmark ----------------------------------
samplers=("ABRs" "Extended bricks")
iterators=("ABRs" "DDA" "MC-BVH" "KDTree" "Brick-BVH", "Extended bricks BVH")

for sampler_mode in {0..1}
do
  for traversal_mode in {0..5}
  do
    cmake ${BUILD_DIR} \
        -DEXA_STITCH_MIRROR_EXAJET=OFF \
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=${sampler_mode} \
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=${traversal_mode}
    cmake --build ${BUILD_DIR} -j

    echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}"

    outfile="-o cloud_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out

    # test with own majorants
    if [ "$traversal_mode" -eq 4 ]
    then
      for m in {10000..10000000}
      do
        outfile="-o cloud_sm${sampler_mode}_tm${traversal_mode}_n${m}"
        majorants="/slow/stitcher/majorants/cloud.n"${m}
        if [ -f "$majorants" ]; then
          echo "Benchmark: ExaBricks, sampler: ${samplers[${sampler_mode}]}, traversal: ${iterators[${traversal_mode}]}, majorants: ${majorants}"
          MAJORANTS="-majorants ${majorants}"
          ${BUILD_DIR}/exaStitchHeadlessViewer ${BRICKS} ${MAJORANTS} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MCS} ${XFORM} ${IMG_SIZE} ${LIGHT} ${outfile} ${fpsfile} -rt 0 2>&1 | tee cloud.out
        fi
      done
    fi
  done
done
