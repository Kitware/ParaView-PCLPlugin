cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)

if (NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
  include(${PARAVIEW_USE_FILE})
endif()

# Set a consistent MACOSX_RPATH default across all CMake versions.
# When CMake 2.8.12 is required, change this default to 1.
# When CMake 3.0.0 is required, remove this block (see CMP0042).
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

include(ParaViewPlugins)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIRS})

set(ServerManagerIncludeDirs
  common
  filters
  sources
  )

set(ServerManagerXML)
set(ServerManagerSources
  common/vtkPCLConversions.cxx
  filters/vtkPCLFilter.cxx
  sources/vtkPCLSource.cxx
  )

# Use a macro to make sure that related files are handled together.
macro(add_filter name)
  list(APPEND ServerManagerXML "xml/${name}.xml")

  string(REGEX MATCH "Source$" filter_type "${name}")
  if ("${filter_type}" STREQUAL "Source")
    list(APPEND ServerManagerSources "sources/vtk${name}.cxx")
  else()
    list(APPEND ServerManagerSources "filters/vtk${name}.cxx")
  endif()
endmacro(add_filter)

# Add the filters.
add_filter(PCLPassThroughFilter)
# add_filter(PCLVoxelGridFilter)
# add_filter(PCLStatisticalOutlierRemovalFilter)
# add_filter(PCLPCDFileSource)


set(plugin_name PCLFilters)
set(plugin_deps ${VTK_LIBRARIES} ${PCL_LIBRARIES})

ADD_PARAVIEW_PLUGIN(
  ${plugin_name} "1.0"
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
)
target_include_directories(${plugin_name} PRIVATE ${ServerManagerIncludeDirs})
target_link_libraries(${plugin_name} LINK_PRIVATE ${plugin_deps})

