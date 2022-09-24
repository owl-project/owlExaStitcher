set SOURCE_DIR=C:\Users\wilso\Documents\Projects\research\exastitch
set BUILD_DIR=C:\Users\wilso\Documents\Projects\research\exastitch\build

set BRICKS=-bricks E:\stitcher\meteor-46112\meteor-46112.bricks
set SCALARS=-scalars E:\stitcher\meteor-46112\meteor-46112.tev.scalars
set KDTREE=-kdtree E:\stitcher\meteor-46112\meteor-46112.kd

set CAMERA=--camera 2607.41 3218.14 3474.3 494.535 1300.13 -210.744 -0.242807 0.91051 -0.334689 -fovy 70
set XF=-xf meteor-46k.xf
@REM set LIGHT=--light -5166.112793 5752.800293 4556.298340 100.000000
set CLIP_PLANE=--clip-plane 0 0 1 921.6
set NUM_MCS=--num-mcs 128 128 64
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

    %BUILD_DIR%\Release\exaStitchHeadlessViewer %BRICKS% %KDTREE% %SCALARS% %MESH% %CAMERA% %XF% %NUM_MCS% %XFORM% %LIGHT% %CLIP_PLANE% %IMG_SIZE% -o meteor-46k_sm%%s_tm%%t -fps meteor-46k_fps.txt -rt 0 > meteor-46k_sm%%s_tm%%t.txt
  )
)
