# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_overlay_pcl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED overlay_pcl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(overlay_pcl_FOUND FALSE)
  elseif(NOT overlay_pcl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(overlay_pcl_FOUND FALSE)
  endif()
  return()
endif()
set(_overlay_pcl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT overlay_pcl_FIND_QUIETLY)
  message(STATUS "Found overlay_pcl: 0.0.0 (${overlay_pcl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'overlay_pcl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${overlay_pcl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(overlay_pcl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${overlay_pcl_DIR}/${_extra}")
endforeach()
