#-------------------------------------------------------------------------------
# add_includes Macro
#-------------------------------------------------------------------------------
# Recursively add include directories to ServerManagerIncludeDirs and sources to
# ServerManagerSources. This ensures that base classes and include directories
# are automatically added to the lists for a given module (filter, source,
# reader or writer) without requiring module developers to manually specify all
# of the files.
macro(add_includes fpath)
  # Regular expressions for parsing local includes and removing extensions (while preserving directory).
  set(include_regex "#include[ \t]+\"([^\"]+)\"")
  set(extension_regex "^(.+)\\.([^.]+)$")

  # The queue of files to be processed.
  list(APPEND fpath_queue "${fpath}")

  # The list of files already processed, to avoid cyclic recursion.
  set(fpath_done)

  # Get the number of items remaining in the queue and loop.
  list(LENGTH fpath_queue fpath_queue_len)

  

  # Loop while items remain in the queue.
  while(${fpath_queue_len} GREATER 0)
    # Pop the first item off the queue.
    list(GET fpath_queue 0 fpath_next)
    list(REMOVE_AT fpath_queue 0)

    # Check if the item has already been processed.
    list(FIND fpath_done "${fpath_next}" found_index)
    if(${found_index} GREATER -1)
      list(LENGTH fpath_queue fpath_queue_len)
      continue()
    endif()

    message(STATUS "parsing includes in ${fpath_next}")

    # Add the directory to the include directories.
    get_filename_component(fpath_dirname "${fpath_next}" DIRECTORY)
    list(APPEND ServerManagerIncludeDirs "${fpath_dirname}")

    # If the file is a source file (cxx or txx extension), add it to
    # ServerManagerSources.
    # LAST_EXT was introduced in 3.14, which is still a release candidate at the time of writing.
    # get_filename_component(fpath_extension "${fpath}" LAST_EXT)
    string(REGEX REPLACE "${extension_regex}" "\\2" fpath_extension "${fpath_next}")
    if("${fpath_extension}" MATCHES "\.?xx$")
      list(APPEND ServerManagerSources "${fpath_next}")
    endif()

    # Add the item to the list of processed items to prevent reprocessing.
    list(APPEND fpath_done "${fpath_next}")

    # Parse include statements from the file using a regex.
    file(STRINGS "${fpath_next}" fpath_includes REGEX "${include_regex}")

    # Process includes.
    if(fpath_includes)
      foreach(tmp_included_item ${fpath_includes})
        # Get the name of the include. If there were a way to iterate over
        # capture groups with MATCHALL, this could be avoided.
        string(REGEX REPLACE "${include_regex}" "\\1" included_item "${tmp_included_item}")

        # Strip the extension from the name. This intentionally preserves the
        # directory, which is not currently possible with any mode of
        # get_filename_component.
        string(REGEX REPLACE "${extension_regex}" "\\1" included_name "${included_item}")

        # Add it and its source files to the queue if they exist.
        # Create the globbing pattern.
        set(included_name_pattern "${CMAKE_SOURCE_DIR}/*/${included_name}.*")
        # Glob the name in the source directory.
        file(
          GLOB included_matches 
          RELATIVE "${CMAKE_SOURCE_DIR}" 
          CONFIGURE_DEPENDS
          "${included_name_pattern}"
        )

        message(STATUS "  - ${included_item}: ${included_matches}")

        # Add the found items to the queue.
        list(APPEND fpath_queue ${included_matches})
      endforeach()
    endif()

    # Update the list length
    list(LENGTH fpath_queue fpath_queue_len)

  endwhile()

endmacro(add_includes)



#-------------------------------------------------------------------------------
# add_module Macro
#-------------------------------------------------------------------------------
# Use a macro to ensure that related files are handled together. The argument is
# a name such as PCLPassThroughFilter or PCLPCDFileReader, without the vtk
# prefix or file extension. The directory will be parsed from the final part of
# the word ("Filter", "Reader", etc.) converted to lower case and suffixed with
# "s". Obviously this macro assumes that all arguments follow the same naming
# conventions.
#
# Additional xml and source files can be passed to this function. They will be
# added ServerManagerXML and ServerManagerSources according to their extension
# (.xml, .cxx, .txx).
macro(add_module name)
  message(STATUS "adding module ${name}")

  # Add the ServerManager xml file.
  list(APPEND ServerManagerXML "xml/${name}.xml")

  # The final part of the name determines the subdirectory in which the C++
  # files are located.  For example, all modules ending in "Filter" are expected
  # to be in the "filter" directory.
  string(REGEX MATCH "[A-Z][a-z]+[0-9]*$" filter_type "${name}")
  string(TOLOWER "${filter_type}" directory)

  # Add the sources and include directories.
  add_includes("${directory}/vtk${name}.cxx")


  # Append any additional xml or source files.
  foreach(additional_arg ${ARGN})
    get_filename_component(additional_arg_ext "${additional_arg}" EXT)
    if("${additional_arg_ext}" STREQUAL ".xml")
      list(APPEND ServerManagerXML "${additional_arg}")
    elseif("${additional_arg_ext}" STREQUAL ".cxx" OR "${additional_arg_ext}" STREQUAL ".txx")
      list(APPEND ServerManagerSources "${additional_arg}")
    else()
      message(WARNING "unrecognized extension passed to add_module macro")
    endif()
  endforeach()
endmacro(add_module)

