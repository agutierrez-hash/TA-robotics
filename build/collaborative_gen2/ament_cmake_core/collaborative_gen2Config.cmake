# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_collaborative_gen2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED collaborative_gen2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(collaborative_gen2_FOUND FALSE)
  elseif(NOT collaborative_gen2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(collaborative_gen2_FOUND FALSE)
  endif()
  return()
endif()
set(_collaborative_gen2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT collaborative_gen2_FIND_QUIETLY)
  message(STATUS "Found collaborative_gen2: 0.0.0 (${collaborative_gen2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'collaborative_gen2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT collaborative_gen2_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(collaborative_gen2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${collaborative_gen2_DIR}/${_extra}")
endforeach()
