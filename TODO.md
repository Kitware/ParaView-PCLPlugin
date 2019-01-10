# TODO

## General

* Use OMP variants for all feature calculations.
* Maybe propagate DataObject pointers to avoid simultaneous handling of PolyData and Table types. The conversion functions could be updated to handle DataObject instances directly (DataObject * <-> PointCloud<PointType>).

## ConvPoint

Split ConvXYZ into a generic base class and template specialization for PointXYZ types in order to support non-XYZ points. Convert non-XYZ points to vtkTables so that all PCL point types can be propagated through the ParaView pipeline.

## OpenNi Source

* Expose all parameters via proxy (RGB & depth camera focal distances and modes).

## PassThrough Filter

* Once non-XYZ points are supported, adapt the filter to accept either a PolyData or a Table instance.

# Bugs

* Normal "downcasting" fails to preserve RGB data.

