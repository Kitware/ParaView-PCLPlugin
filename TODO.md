# TODO

## General

* Create [vtkMultiBlockDataSetAlgorithm](https://vtk.org/doc/nightly/html/classvtkMultiBlockDataSetAlgorithm.html) base classes for filters that return multiple inputs (e.g. the pending Euclidean cluster extraction filter).
* Consider how to improve the directory hierarchy.
* Add KdTree options to filters that support them (use PCLNormalEstimationFilter as template).

## OpenNi Source

* Expose all parameters via proxy (RGB & depth camera focal distances and modes).

## vtkPCLWriter

Remove `FileName` and related attributes once they are included in `vtkWriter`.

