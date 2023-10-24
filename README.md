Exa Stitching Renderer
======================

Prototype renderer that can handle volumetric bricks ("gridlets") plus umeshes,
such as they occur if one generates a dual AMR grid, stitches the boundary
regions with unstructured elements, but clusters and then represents the
interior of the duals with grid primitives.

Usage
=====

```
./exaStitchViewer boundaryCells.umesh -grids gridlets.grids -scalars scalars.bin
```

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

Volume Lines Extension
======================

This branch contains extension for the paper
"Visual Analysis of Large Multi-Field AMR Data on GPUs Using Interactive Volume Lines"
Code for that can be found in the folder `va`.
