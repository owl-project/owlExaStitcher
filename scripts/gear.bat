set SOURCE_DIR=C:\Users\wilso\Documents\Projects\research\exastitch
set BUILD_DIR=C:\Users\wilso\Documents\Projects\research\exastitch\build

set BRICKS=-bricks E:\stitcher\landinggear\gear.bricks
set SCALARS=-scalars E:\stitcher\landinggear\chombo.exa.velmag.scalars
set KDTREE=-kdtree E:\stitcher\landinggear\gear.kd
set MESH=-mesh E:\stitcher\landinggear\landingGearThickFilledGap.tris

set CAMERA=--camera -0.1688781381 0.7803725004 0.2884992957 19.29099655 -20.87918472 -2.018322945 0 0 -1 --fov 40
set XFORM=--remap-from 0 0 0 131071 131071 65535 --remap-to -16 -16 -.1 16 16 16
set IMG_SIZE=-win 1024 1024
set NUM_MCS=--num-mcs 1024 1024 256
set XF=-xf gear.xf

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

    %BUILD_DIR%\Release\exaStitchHeadlessViewer %BRICKS% %KDTREE% %SCALARS% %MESH% %CAMERA% %XF% %NUM_MCS% %XFORM% %LIGHT% %CLIP_PLANE% %IMG_SIZE% -o gear_sm%%s_tm%%t -fps gear_fps.txt -rt 0 > gear_sm%%s_tm%%t.txt
  )
)
