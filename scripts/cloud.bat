set SOURCE_DIR=C:\Users\wilso\Documents\Projects\research\exastitch
set BUILD_DIR=C:\Users\wilso\Documents\Projects\research\exastitch\build

set BRICKS=-bricks E:\stitcher\silcc\SILCC_hdf5_plt_cnt_0300_dens.bricks
set SCALARS=-scalars E:\stitcher\silcc\SILCC_hdf5_plt_cnt_0300_dens.scalars
set KDTREE=-kdtree E:\stitcher\silcc\SILCC_hdf5_plt_cnt_0300_dens.kd

set CAMERA=--camera 5346.1 1520.14 82012.8 6671.5 2211.75 81920.5 -0.430131 0.759096 -0.488631 -fovy 70
set XF=-xf cloud.xf
@REM set LIGHT="--light -13885.075195 3776.682129 86940.859375 900.000000"
set NUM_MCS=--num-mcs 128 128 128
set IMG_SIZE=-win 1024 1024

FOR /L %%s IN (0,1,1) DO (
  FOR /L %%t IN (0,1,4) DO (
    cmake -B %BUILD_DIR% -S %SOURCE_DIR%  -G "Visual Studio 17 2022" -T host=x64 -A x64 ^
        -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=OFF ^
        -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=ON ^
        -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=OFF ^
        -DEXA_STITCH_MIRROR_EXAJET=OFF ^
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=%%s ^
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=%%t ^
        -DBUILD_SHARED_LIBS=OFF ^
        -DOptiX_INSTALL_DIR="C:/ProgramData/NVIDIA Corporation/OptiX SDK 7.4.0" 

    cmake --build %BUILD_DIR% --config Release

    %BUILD_DIR%\Release\exaStitchHeadlessViewer %BRICKS% %KDTREE% %SCALARS% %MESH% %CAMERA% %XF% %NUM_MCS% %XFORM% %LIGHT% %CLIP_PLANE% %IMG_SIZE% -o cloud_sm%%s_tm%%t -fps cloud_fps.txt -rt 0 > cloud_sm%%s_tm%%t.txt
  )
)
