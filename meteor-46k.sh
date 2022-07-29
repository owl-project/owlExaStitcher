#CAMERA="--camera -0.1688781381 0.7803725004 0.2884992957 19.29099655 -20.87918472 -2.018322945 0 0 -1 -fovy 40"
CAMERA=""
XF="-xf meteor-46k.xf"

./exaStitchViewer /slow/stitcher/meteor-46k.umesh -grids /slow/stitcher/meteor-46k.grids -scalars /slow/UNSORTED/exa/meteor-46k/meteor-46112.tev.scalars  $CAMERA $XF
