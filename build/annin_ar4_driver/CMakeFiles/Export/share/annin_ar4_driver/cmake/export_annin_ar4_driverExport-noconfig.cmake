#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "annin_ar4_driver::annin_ar4_driver" for configuration ""
set_property(TARGET annin_ar4_driver::annin_ar4_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(annin_ar4_driver::annin_ar4_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libannin_ar4_driver.so"
  IMPORTED_SONAME_NOCONFIG "libannin_ar4_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS annin_ar4_driver::annin_ar4_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_annin_ar4_driver::annin_ar4_driver "${_IMPORT_PREFIX}/lib/libannin_ar4_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
