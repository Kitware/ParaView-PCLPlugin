cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)

#-------------------------------------------------------------------------------
# Find ParaView
if(NOT ParaView_BINARY_DIR)
  find_package(ParaView REQUIRED)
  message(STATUS "PARAVIEW_USE_FILE: ${PARAVIEW_USE_FILE}")
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
find_package(ospray)
if(ospray_FOUND)
  link_directories(${ospray_LIBRARY_DIRS})
endif()

#-------------------------------------------------------------------------------
# Options
option(ENABLE_OPENNI "Enable OpenNI Support (requires PCL with OpenNI support)." OFF)

#-------------------------------------------------------------------------------
# Initialize ParaView plugin variables.
set(ServerManagerIncludeDirs)
set(ServerManagerSources)
set(ServerManagerXML xml/PCLCommon.xml)


#-------------------------------------------------------------------------------
# Include the "add_module" macro.
include("cmake/macros.cmake")

#-------------------------------------------------------------------------------
# Add the plugin modules (filters, readers, etc.) The add_module macro accepts
# the module name (e.g. PCLPassThroughFilter), adds the xml and source files,
# and ensures that the include directory and the base class's source files have
# already been added. Additional xml and/or source files may be passed after the
# name. This is necessary for txx files to ensure that the expected
# client-server files are generated.

# General
# add_module(PCLDummyFilter)

# Conversions
add_module(PCLPointCloudXYZRGBtoXYZIFilter)

# Generic filters.
add_module(PCLPassThroughFilter)
add_module(PCLStatisticalOutlierRemovalFilter)
add_module(PCLVoxelGridFilter)

# Readers, writers and sources.
add_module(PCLPCDFileReader)
add_module(PCLPCDFileWriter)

if(ENABLE_OPENNI)
  if(OpenNI_FOUND OR OpenNI2_FOUND)
    add_module(PCLOpenNISource)
  else()
    message(WARNING "OpenNI was not found.")
  endif()
endif()

# Features
add_module(PCLNormalEstimationFilter)
add_module(PCLFPFHEstimationFilter2)
add_module(PCLRSDEstimationFilter2)
add_module(PCLIntensityGradientEstimationFilter2)

# Misc
add_module(PCLMovingLeastSquaresFilter)

# Registration/alignment
add_module(PCLIterativeClosestPointFilter2 filter2/vtkPCLRegistrationFilter2.txx)
# add_module(PCLSampleConsensusInitialAlignmentFilter2 filter2/vtkPCLRegistrationFilter2.txx)

# Custom/compound.
add_module(PCLBSampleConsensusModelRegistrationFilter4)
add_module(PCLBRigidAlignmentFilter2)

#-------------------------------------------------------------------------------
list(REMOVE_DUPLICATES ServerManagerIncludeDirs)
list(REMOVE_DUPLICATES ServerManagerSources)
list(REMOVE_DUPLICATES ServerManagerXML)

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

