Exa Stitching Renderer
======================

![teaser](/img/teaser.jpg "Teaser")

Overview
========

Prototype renderer that can handle volumetric bricks ("gridlets") plus umeshes,
such as they occur if one generates a dual AMR grid, stitches the boundary
regions with unstructured elements, but clusters and then represents the
interior of the duals with grid primitives.

This project contains sample code for the research paper presented at EuroVis 2023
(Computer Graphics Forum):

Stefan Zellmann, Qi Wu, Kwan-Liu Ma, Ingo Wald:\
"_Memory-Efficient GPU Volume Path Tracing of AMR Data Using the Dual Mesh_" \
Computer Graphics Forum, Vol. 42, Issue 3, 2023 (Proceedings of EuroVis 2023)

Citation:
```
@article {10.1111:cgf.14811,
journal = {Computer Graphics Forum},
title = {{Memory-Efficient GPU Volume Path Tracing of AMR Data Using the Dual Mesh}},
author = {Zellmann, Stefan and Wu, Qi and Ma, Kwan-Liu and Wald, Ingo},
year = {2023},
publisher = {The Eurographics Association and John Wiley & Sons Ltd.},
ISSN = {1467-8659},
DOI = {10.1111/cgf.14811}
}
```

Author version: https://pds.uni-koeln.de/sites/pds/szellma1/eurovis23-stitcher-author-version-compressed.pdf

High-res author version: https://pds.uni-koeln.de/sites/pds/szellma1/eurovis23-stitcher-author-version.pdf

Official version: https://diglib.eg.org/handle/10.1111/cgf14811

Usage
=====

This sample code was tested on Ubuntu 22.04, with `nvcc` 11.4 and OptiX 7.4.

(Boundary) cells are generated with a separate tool (see
[tools/amrMakeDualMesh](/tools/amrMakeDualMesh). First use `amrMakeDualMesh` to
construct the dual mesh. The "real" unstructured cells end up in a `.umesh`
file, the remaining files in `cube` files that are subsequently clustered to
`.grids` files using the `amrMakeGrids` tool. These include the gridlets from
the paper.

The project includes an interactive renderer:

```
./exaStitchViewer boundaryCells.umesh -grids gridlets.grids -scalars scalars.bin
```

# Usage on Windows

Example:
```
mkdir build
cd build
cmake .. -DOptiX_INSTALL_DIR="C:/ProgramData/NVIDIA Corporation/OptiX SDK 7.4.0" -DQt5Widgets_DIR="C:/Users/wilso/Documents/Softwares/win/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Widgets" -DBUILD_SHARED_LIBS=OFF
cmake --build . --config Release
```
i.e., don't build shared libs; otherwise similar to Linux.

General Notes
=============

The default file type interpreted by the viewer is `umesh`; the viewer will
build a sampling BVH over the unstructured elements contained inside and use
that to implement volume sampling with OptiX/RTX. Passing an `umesh` file is
mandatory for now, as we assume that we'll always need unstructured boundary
elements.

Note that it's perfectly valid for the `umesh` file to also contain all the
_voxels_/perfect cubes of the AMR hierarchy and represent them as hexes; that
way, the dual mesh is fully represented with unstructured elements, as one
would e.g. do in a traditional AMR stitching scenario.

Alternatively, the voxels may also come from a `grids` file (cmdline option
`-grids`), obtained by generating and then clustering the dual grid with the
`amrMakeDual` tool. We assume that the `umesh` then doesn't contain these
voxels/hexes, as they come from the `grids` file.

The `umesh` file is allowed to either include per-vertex scalar values
(`float32`), or cell IDs that point into the scalars file (cmdline option
`-scalars`). `grids` files are however always store scalar IDs, i.e., passing a
`grids` file without scalars will result in rendering being broken because the
scalars are missing

Code Walkthrough
================

The GPU rendering code is primarily implemented in
[deviceCode.cu](/deviceCode.cu). Here a ray gen program is implemented, plus
path tracers using Woodcock tracking, and a BVH traversal mode for majorants.
During development we tested different majorant traversal modes and ultimately
stuck with a DDA grid (see [DDA.h](/DDA.h) and [DDA.cu](/DDA.cu)) (traversal is
initiate in [deviceCode.cu](/deviceCode.cu).

The OptiX intersection and closest hit programs used for _sampling_ with
zero-length rays live in separate samplers, implementations of which can be
found in [sampler](/sampler). Samplers are associated with _models_ (see
[model](/model)). When loading a model from file, the respective (GPU) sampler
will automatically be constructed later on.

The sampler presented in the paper is the `ExaStitch` sampler. More examples
include the `ExaBrick` sampler (Wald et al. (2020) "_Ray Tracing Structured AMR
Data Using ExaBricks_", source code for this sampler is included), the `BigMesh`
sampler (source code **not** included, Wald et al. (2021) "_A Memory Efficient
Encoding for Ray Tracing Large Unstructured Data_"), and the `QuickClusters`
sampler (Morrical et al. 2022, "_Quick Clusters: A GPU-Parallel Partitioning for
Efficient Path Tracing of Unstructured Volumetric Grids_", source code
included). These other samplers are used for comparison in the paper, and the
source code was adapted from the original source code developed by the authors
and reused with friendly permission. For the `BigMesh` sampler we also used the
original implementation, yet cannot provide source code here due to licensing
issues.

Adding additional model/sampler pairs should be straightforward. Pointers how
to invoke the various models/samplers can be found in the [scripts](/scripts)
directory. To generate "ExaBricks" files, the command line tool chain from the
[original repository](https://github.com/owl-project/owlExaBrick) must be used.

The CPU host code to invoke the GPU renderer is primarily implemented in
[OWLRenderer.h](/OWLRenderer.h) and [OWLRenderer.cpp](/OWLRenderer.cpp).

ExaStitch model files are generated using an offline toolchain (see the
description above under "Usage").

The code includes a prototypical "anari" device; to be used with the
[ANARI-SDK](https://github.com/KhronosGroup/ANARI-SDK). This code is under
development, but will eventually allow us to integrate the data structure with
VTK or VTK-m. Example code how to use the device and anari extension can be
found under [anari/example.md](/anari/example.md).

External Dependencies
=====================

We use the following external dependencies (included in the
[submodules](/submodules) directory):

- [OWL](https://github.com/owl-project/owl): the OptiX 7 wrapper library,
  providing easy access to the OptiX library, by Ingo Wald
- [UMesh](https://gitlab.com/ingowald/umesh): library to represent unstructured
  element meshes on the host and file system, by Ingo Wald
- [cuteeOwl](https://github.com/owl-project/cuteeOwl): QT GUI providing
  transfer function editing, by Ingo Wald
- [Visionaray](https://github.com/szellmann/visionaray): is used for a software
  BVH implementation; included here, but only relevant for building offline
  kd-trees with majorant (not discussed in the paper)

People Involved
===============

- Stefan Zellmann, Qi Wu, Kwan-Liu Ma, and Ingo Wald
- Nate Morrical (provided code for the "QuickClusters" sampler)

License
=======

Apache 2, if not noted otherwise. See the [LICENSE](/LICENSE) file.
