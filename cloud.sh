#CAMERA="--camera 6552.13 2489.99 82274.5 6671.5 2211.75 81920.5 0 1 0 -fovy 70"
CAMERA="--camera 6696.54 2817.33 83468.8 6671.5 2211.75 81920.5 0 1 0 -fovy 70"
XF="-xf cloud.xf"

./exaStitchViewer /slow/stitcher/cloud.umesh -grids /slow/stitcher/cloud.grids -scalars /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.scalars $CAMERA $XF
#./exaStitchViewer -cells /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.cells -scalars /fast/data/exa/MC1_L11/SILCC_hdf5_plt_cnt_0300_dens.scalars $CAMERA $XF
