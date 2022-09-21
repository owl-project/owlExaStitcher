set SOURCE_DIR=C:\Users\wilso\Documents\Projects\research\exastitch
set BUILD_DIR=C:\Users\wilso\Documents\Projects\research\exastitch\build

set BRICKS=-bricks E:\stitcher\exajet\exajet-spatial-32min.bricks
set SCALARS=-scalars E:\stitcher\exajet\mag_vorticity.bin
set KDTREE=-kdtree E:\stitcher\exajet\exajet-spatial-32min.kdtree
set MESH=-mesh E:\stitcher\exajet\exajet.exa.tris

set CAMERA=--camera 3.685920477 -1.043313146 0.7212070227 14.80596733 -3.935504913 -3.436543465 0 0 1
set XF=-xf exajet-wing.xf
set NUM_MCS=--num-mcs 512 512 512
set XFORM=--remap-from 1232128 1259072 1238336 1270848 1277952 1255296 --remap-to -1.73575 -9.44 -3.73281 17.6243 0 4.74719
set IMG_SIZE=--size 1024 1024

FOR /L %%s IN (0,1,1) DO (
  FOR /L %%t IN (0,1,4) DO (
    cmake -B %BUILD_DIR% -S %SOURCE_DIR%  -G "Visual Studio 17 2022" -T host=x64 -A x64 ^
        -DEXA_STITCH_WITH_EXA_STITCH_SAMPLER=OFF ^
        -DEXA_STITCH_WITH_EXA_BRICK_SAMPLER=ON ^
        -DEXA_STITCH_WITH_AMR_CELL_SAMPLER=OFF ^
        -DEXA_STITCH_MIRROR_EXAJET=ON ^
        -DEXA_STITCH_EXA_BRICK_SAMPLER_MODE=%%s ^
        -DEXA_STITCH_EXA_BRICK_TRAVERSAL_MODE=%%t ^
        -DBUILD_SHARED_LIBS=OFF ^
        -DOptiX_INSTALL_DIR="C:/ProgramData/NVIDIA Corporation/OptiX SDK 7.4.0" 

    cmake --build %BUILD_DIR% --config Release

    %BUILD_DIR%\Release\exaStitchHeadlessViewer %BRICKS% %KDTREE% %SCALARS% %MESH% %CAMERA% %XF% %NUM_MCS% %XFORM% %LIGHT% %CLIP_PLANE% %IMG_SIZE% -o exajet-wing_sm%%s_tm%%t -rt 0 > exajet-wing_sm%%s_tm%%t.txt
  )
)
