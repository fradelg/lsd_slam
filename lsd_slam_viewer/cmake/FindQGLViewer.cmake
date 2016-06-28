FIND_PATH(QGLVIEWER_INCLUDE_DIR qglviewer.h
  /usr/local/include/QGLViewer
  /usr/include/QGLViewer
  /opt/local/include/QGLViewer
  /sw/include/QGLViewer
  ENV QGLVIEWERROOT
)

message(STATUS "Found QGLViewer: ${QGLVIEWER_INCLUDE_DIR}")

find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES qglviewer QGLViewer QGLViewer2
  PATHS /usr/local/lib
        /usr/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
)

find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES dqglviewer QGLViewerd dQGLViewer2 QGLViewerd2
  PATHS /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
  PATH_SUFFIXES QGLViewer QGLViewer/release
)

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QGLVIEWER DEFAULT_MSG
  QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)
