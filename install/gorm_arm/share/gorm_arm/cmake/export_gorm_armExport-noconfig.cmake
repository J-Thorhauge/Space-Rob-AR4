#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gorm_arm::gorm_arm" for configuration ""
set_property(TARGET gorm_arm::gorm_arm APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(gorm_arm::gorm_arm PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgorm_arm.so"
  IMPORTED_SONAME_NOCONFIG "libgorm_arm.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gorm_arm::gorm_arm )
list(APPEND _IMPORT_CHECK_FILES_FOR_gorm_arm::gorm_arm "${_IMPORT_PREFIX}/lib/libgorm_arm.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
