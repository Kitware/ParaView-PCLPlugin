# Introduction

This plugin aims to bring the full functionality of the [Point Cloud Library (PCL)](http://www.pointclouds.org/) to [ParaView](https://www.paraview.org/) in an easily extensible class hierarchy. The core of the plugin provides a class to convert PCL point clouds of any point type to and from VTK PolyData instances so that data can be passed seamlessly through a full ParaView workflow. This communication with ParaView is mostly handled by common base classes so that developers can focus on transforming point clouds.

# Installation

To build, install the required dependencies then change to the source directory and run the following commands:

~~~~~{.sh}
mkdir -p build
cd build
cmake ..
cmake --build .
~~~~~

## Dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency  | Tested Version  | Minimum Version  |
| :---------: | :-------------: | :--------------: |
| ParaView    | master          | ?                |
| PCL         | 1.9             | ?                |
| Eigen       | 3.6.3           | ?                |
| Boost       | 1.68            | ?                |

The dependency on ParaView master is temporary and due to a minor issue with ParaView 5.6.0 that will be corrected in the next release (normally spring 2019).

### PCL Configuration

If PCL is compiled against VTK, segfaults may occur during plugin loading. Strangely, the error only appears during manual loading. To prevent this, either replace `find_package(VTK)` with `find_package(ParaView)` in PCL's CMakeLists.txt file or pass the following options to CMake when building PCL:

* `-DWITH_VTK:BOOL=OFF`
* `-DBUILD_visualization:BOOL=OFF`

To enable OpenNI support, PCL must also be compiled with `-DWITH_OPENNI:BOOL=ON` or `-DWITH_OPENNI2:BOOL=ON` and the corresponding OpenNI library must be installed.

# Developers

## Basic

`PCLPassThroughFilter` has been commented to provide a starting point for the development of new filters. In general, a new filter will only require 3 files to be copied and modified from one of the existing filters: the VTK class source and header files and the server manager proxy XML file (e.g. filter/vtkPCLPassThroughFilter.h, filter/vtkPCLPassThroughFilter.cxx, xml/PCLPassThroughFilter.xml).

### VTK Header

* Declare minimal VTK boilerplate code (copy-paste from existing filters and update the names).
* Declare the parameters as members of the VTK class along with setters and getters using the standard VTK macros (e.g. `vtkGetMacro`, `vtkSetMacro`, `vtkSetVector2Macro`, ...).
* Declare the private method invoked by the parent class's RequestData method (`ApplyPCLFilter` for filters, `LoadPCLReader` for readers, etc.)
* Declare any additional private internal methods needed by the aforementioned method.

### VTK Source

* Define minimal VTK boilerplate code (copy-paste from existing filters and update the names).
* Define the other declared methods. The general approach is to use vtkPCLConversions to determine the point type index of the input PolyData instance and then invoke a templated method that accepts the point type as a template parameter to handle the actual conversions. This is much simpler than it may initially sound and requires only 4 lines of code. The templated function then handles the actual conversion. This approach can be nested for multiple inputs of arbitrary PCL point type. The various `PCLP_INVOKE_WITH_*_POINT_TYPE` macros are provided for this purpose.

### Server Manager Proxy

* Expose and document the desired parameters of the filter and provide default values. These should match the names and values declared in the header, which in turn should match the names used in PCL.
* Add the filter to the PCL category, set the number of inputs, etc.

This usually just means copy-pasting an existing proxy and updating its name, documentation and parameters.

## Beyond Basic

It is possible to create filters with multiple inputs, some of which may even be optional. The `FPFHEstimationFilter2` provides an example of this. The same approach can and will be extended to filters with more inputs as the library grows. Although not yet implemented, it is also possible to create VTK filters with multiple outputs. Developers that wish to add new filters for which there currently is no base class (e.g. `vtkPCLFilter3` at the time of writing) should create the base class using the others as a model.

Ideally each filter should correspond as closely as possible to a single PCL functionality (e.g. `PassThrough`, `NormalEstimation`, `VoxelGrid`). In such cases, the name of the filter and class should use the PCL filter to signify that it is essentially a pure PCL function. In some cases this is nevertheless impossible, such as when intermediate steps and PolyData-inconvertible data objects are required. Filters that involve intermediate steps should use the PCLB prefix to indicate that they are PCL-based but not pure PCL). All internal parameters should be exposed via the server manager proxy to make the filter as generic as possible.

