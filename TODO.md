# TODO

## General

* Create [vtkMultiBlockDataSetAlgorithm](https://vtk.org/doc/nightly/html/classvtkMultiBlockDataSetAlgorithm.html) base classes for filters that return multiple inputs (e.g. the pending Euclidean cluster extraction filter).
* Consider how to optimize the directory hierarchy.
* Update the CMake `add_module` macro to allow for more flexibility (e.g. remove the assumption of a particular base class through file inspection).

## OpenNi Source

* Expose all parameters via proxy (RGB & depth camera focal distances and modes).

## vtkPCLWriter

Remove `FileName` and related attributes once they are included in `vtkWriter`.

