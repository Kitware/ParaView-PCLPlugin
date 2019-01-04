cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)

#-------------------------------------------------------------------------------
# Find ParaView
if (NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
  include(${PARAVIEW_USE_FILE})
endif()
include(ParaViewPlugins)

#-------------------------------------------------------------------------------
# Find PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#-------------------------------------------------------------------------------
# Find Boost
find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIRS})
# link_directories(${BOOST_LIBRARY_DIRS})

#-------------------------------------------------------------------------------
# Options
option(ENABLE_OPENNI "Enable OpenNI Support (requires PCL with OpenNI support)." OFF)

#-------------------------------------------------------------------------------
# Initialize ParaView plugin variables.
set(ServerManagerIncludeDirs
  common
  )
set(ServerManagerSources
  common/vtkPCLConversions.cxx
  )
set(ServerManagerXML)

#-------------------------------------------------------------------------------
# Macro
#-------------------------------------------------------------------------------
# Append an item to a list if not already in the list.
macro(append_if_missing list item)
  list(FIND "${list}" "${item}" item_index)
  if(${item_index} EQUAL -1)
    list(APPEND "${list}" "${item}")
  endif()
endmacro(append_if_missing)

#-------------------------------------------------------------------------------
# Macro
#-------------------------------------------------------------------------------
# Use a macro to ensure that related files are handled together. The argument is
# a name such as PCLPassThroughFilter or PCLPCDFileReader, without the vtk
# prefix or file extension. The directory will be parsed from the final part of
# the word ("Filter", "Reader", etc.) converted to lower case and suffixed with
# "s". Obviously this macro assumes that all arguments follow the same naming
# conventions.
macro(add_module name)
  message(STATUS "Add module ${name}.")
  list(APPEND ServerManagerXML "xml/${name}.xml")

  string(REGEX MATCH "[A-Z][a-z]+[0-9]*$" filter_type "${name}")
  string(TOLOWER "${filter_type}" directory)

  # Ensure that the include directory is included.
  append_if_missing(ServerManagerIncludeDirs "${directory}")
  # Ensure that the parent class source is included, e.g. vtkPCLFilter.cxx for
  # all filter types.
  append_if_missing(ServerManagerSources "${directory}/vtkPCL${filter_type}.cxx")

  list(APPEND ServerManagerSources "${directory}/vtk${name}.cxx")
endmacro(add_module)

#-------------------------------------------------------------------------------
# Add the plugin modules (filters, readers, etc.) The add_module macro will add
# the XML and source files, and ensure that the include directory and the base
# class's source files have already been added.

# add_module(PCLDummyFilter)
add_module(PCLPassThroughFilter)
add_module(PCLVoxelGridFilter)
add_module(PCLStatisticalOutlierRemovalFilter)
add_module(PCLPCDFileReader)
add_module(PCLPCDFileWriter)

if(ENABLE_OPENNI)
  if(OpenNI_FOUND OR OpenNI2_FOUND)
    add_module(PCLOpenNISource)
  else()
    message(WARNING "OpenNI was not found.")
  endif()
endif()

#-------------------------------------------------------------------------------
# Function
#-------------------------------------------------------------------------------
# Print the name of a list followed its sorted members on separate indented
# lines.
function(print_list list_name)
  message(STATUS "${list_name}")
  set(tmp_list "${${list_name}}")
  list(SORT tmp_list)
  foreach(item ${tmp_list})
    message(STATUS "  ${item}")
  endforeach()
endfunction(print_list)

#-------------------------------------------------------------------------------
# Print some status information.
print_list(ServerManagerXML)
print_list(ServerManagerSources)
print_list(ServerManagerIncludeDirs)

#-------------------------------------------------------------------------------
set(plugin_name PCLFilters)
set(plugin_deps ${VTK_LIBRARIES} ${PCL_LIBRARIES})

ADD_PARAVIEW_PLUGIN(
  ${plugin_name} "1.0"
  SERVER_MANAGER_XML ${ServerManagerXML}
  SERVER_MANAGER_SOURCES ${ServerManagerSources}
)
target_include_directories(${plugin_name} PRIVATE ${ServerManagerIncludeDirs})
target_link_libraries(${plugin_name} LINK_PRIVATE ${plugin_deps})

