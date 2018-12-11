cmake_minimum_required(VERSION 3.3)

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

set(PCLFilterXML PCLPassThroughFilter.xml)
set(PCLFilterSources
  vtkPCLConversions.cxx
  vtkPCLPassThroughFilter.cxx
)

set(library_name PCLFilters)
# set(server_deps ${VTK_LIBRARIES} ${PCL_LIBRARIES})

ADD_PARAVIEW_PLUGIN(
  ${library_name} "1.0"
  SERVER_MANAGER_XML ${PCLFilterXML}
  SERVER_MANAGER_SOURCES ${PCLFilterSources}
)
# target_link_libraries(${library_name} LINK_PUBLIC ${server_deps})

