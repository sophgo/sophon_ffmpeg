# ----------------------------------------------------------------------------
#  CMake file for libpng. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------

if(ENABLE_NEON)
  project(${PNG_LIBRARY} C ASM)
else()
  project(${PNG_LIBRARY} C)
endif()

if(NOT WIN32)
  find_library(M_LIBRARY
    NAMES m
    PATHS /usr/lib /usr/local/lib
  )
  if(NOT M_LIBRARY)
    message(STATUS "math lib 'libm' not found; floating point support disabled")
  endif()
else()
  # not needed on windows
  set(M_LIBRARY "")
endif()

ocv_include_directories("${CMAKE_CURRENT_SOURCE_DIR}" ${ZLIB_INCLUDE_DIRS})

file(GLOB lib_srcs *.c)
file(GLOB lib_hdrs *.h)


if(ARM OR AARCH64)
  if(ENABLE_NEON)
    if(NOT AARCH64)
      list(APPEND lib_srcs arm/filter_neon.S)
    endif()
    list(APPEND lib_srcs arm/arm_init.c arm/filter_neon_intrinsics.c arm/palette_neon_intrinsics.c)
    add_definitions(-DPNG_ARM_NEON_OPT=2)
    add_definitions(-DPNG_ARM_NEON_API_SUPPORTED)
    add_definitions(-DPNG_ARM_NEON_CHECK_SUPPORTED)
  else()
    add_definitions(-DPNG_ARM_NEON_OPT=0) # NEON assembler is not supported
  endif()
endif()

if(";${CPU_BASELINE_FINAL};" MATCHES "SSE2"
    AND (NOT MSVC OR (MSVC_VERSION GREATER 1799))) # MSVS2013+ (issue #7232)
  list(APPEND lib_srcs intel/intel_init.c intel/filter_sse2_intrinsics.c)
  add_definitions(-DPNG_INTEL_SSE)
endif()

if(PPC64LE OR PPC64)
  if(ENABLE_VSX AND NOT PPC64)
    list(APPEND lib_srcs powerpc/powerpc_init.c powerpc/filter_vsx_intrinsics.c)
    add_definitions(-DPNG_POWERPC_VSX_OPT=2)
  else()
    add_definitions(-DPNG_POWERPC_VSX_OPT=0)
  endif()
endif()

# ----------------------------------------------------------------------------------
#         Define the library target:
# ----------------------------------------------------------------------------------

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif(MSVC)

add_library(${PNG_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})
target_link_libraries(${PNG_LIBRARY} ${ZLIB_LIBRARIES})

ocv_warnings_disable(CMAKE_C_FLAGS -Wundef -Wcast-align -Wimplicit-fallthrough -Wunused-parameter -Wsign-compare)

set_target_properties(${PNG_LIBRARY}
  PROPERTIES OUTPUT_NAME ${PNG_LIBRARY}
  DEBUG_POSTFIX "${OPENCV_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${PNG_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${PNG_LIBRARY}${OPENCV_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${3P_LIBRARY_OUTPUT_PATH}
  )

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${PNG_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

if(NOT BUILD_SHARED_LIBS)
  ocv_install_target(${PNG_LIBRARY} EXPORT OpenCVModules ARCHIVE DESTINATION ${OPENCV_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()

ocv_install_3rdparty_licenses(libpng LICENSE README)
