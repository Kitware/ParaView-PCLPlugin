# TODO

## General

* Create [vtkMultiBlockDataSetAlgorithm](https://vtk.org/doc/nightly/html/classvtkMultiBlockDataSetAlgorithm.html) base classes for filters that return multiple inputs (e.g. the pending Euclidean cluster extraction filter).
* Consider how to improve the directory hierarchy.
* Add KdTree options to filters that support them (use PCLNormalEstimationFilter as template).

## Create a pcl::Feature Class

A number of PCL filter classes derive from [pcl::Feature](http://docs.pointclouds.org/trunk/classpcl_1_1_feature.html) and thus share several parameters. A common class and base proxy should be introduced to share these parameters across several filters. The main difficulty is the lack of support for multiple inheritance and templated inheritance in VTK. As only a linear hierarchy is possible, the possible solutions are not ideal. This is yet another reason to reconsider an approach that generates filters automatically from simple configuration files.

## OpenNi Source

* Expose all parameters via proxy (RGB & depth camera focal distances and modes).

## vtkPCLWriter

Remove `FileName` and related attributes once they are included in `vtkWriter`.

