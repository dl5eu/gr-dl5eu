find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_DL5EU gnuradio-dl5eu)

FIND_PATH(
    GR_DL5EU_INCLUDE_DIRS
    NAMES gnuradio/dl5eu/api.h
    HINTS $ENV{DL5EU_DIR}/include
        ${PC_DL5EU_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_DL5EU_LIBRARIES
    NAMES gnuradio-dl5eu
    HINTS $ENV{DL5EU_DIR}/lib
        ${PC_DL5EU_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-dl5euTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_DL5EU DEFAULT_MSG GR_DL5EU_LIBRARIES GR_DL5EU_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_DL5EU_LIBRARIES GR_DL5EU_INCLUDE_DIRS)
