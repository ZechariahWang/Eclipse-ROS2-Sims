# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_eclipse_ackermann_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED eclipse_ackermann_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(eclipse_ackermann_FOUND FALSE)
  elseif(NOT eclipse_ackermann_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(eclipse_ackermann_FOUND FALSE)
  endif()
  return()
endif()
set(_eclipse_ackermann_CONFIG_INCLUDED TRUE)

# output package information
if(NOT eclipse_ackermann_FIND_QUIETLY)
  message(STATUS "Found eclipse_ackermann: 0.0.1 (${eclipse_ackermann_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'eclipse_ackermann' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${eclipse_ackermann_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(eclipse_ackermann_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${eclipse_ackermann_DIR}/${_extra}")
endforeach()
