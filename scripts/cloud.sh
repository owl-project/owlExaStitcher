BUILD_DIR="/home/ampere/stefan/exastitch/build"
BRICKS="-bricks /slow/qiwu/SILCC_hdf5_plt_cnt_0300_dens.bricks"
SCALARS="-scalars /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.scalars"
KDTREE="-kdtree /slow/qiwu/SILCC_hdf5_plt_cnt_0300_dens.kd"

CAMERA="--camera 5346.1 1520.14 82012.8 6671.5 2211.75 81920.5 -0.430131 0.759096 -0.488631 -fovy 70"
XF="-xf cloud.xf"
LIGHT="--light -13885.075195 3776.682129 86940.859375 900.000000"
NUM_MCS="--num-mcs 128 128 128"
IMG_SIZE="-win 1024 1024"

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

    outfile="-o meteor-46k_sm${sampler_mode}_tm${traversal_mode}"
    ${BUILD_DIR}/exaStitchViewer ${BRICKS} ${KDTREE} ${SCALARS} ${MESH} ${CAMERA} ${XF} ${NUM_MS} ${LIGHT} ${CLIP_PLANE} ${IMG_SIZE} ${outfile} -rt 0 2>&1 | tee meteor-46k.out
  done
done
