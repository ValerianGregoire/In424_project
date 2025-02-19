# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_in424_desc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED in424_desc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(in424_desc_FOUND FALSE)
  elseif(NOT in424_desc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(in424_desc_FOUND FALSE)
  endif()
  return()
endif()
set(_in424_desc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT in424_desc_FIND_QUIETLY)
  message(STATUS "Found in424_desc: 1.0.0 (${in424_desc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'in424_desc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${in424_desc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(in424_desc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${in424_desc_DIR}/${_extra}")
endforeach()
