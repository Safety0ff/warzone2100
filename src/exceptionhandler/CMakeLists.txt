set (exceptionhandler_HEADERS
  dumpinfo.h
  exceptionhandler.h
  include/coff/internal.h
  include/demangle.h
  include/libcoff.h
  CACHE INTERNAL ""
)

set (exceptionhandler_SOURCES
  dumpinfo.cpp
  exceptionhandler.cpp
  CACHE INTERNAL ""
)

if (WIN32 OR MINGW)
  set (exceptionhandler_HEADERS ${exceptionhandler_HEADERS} exchndl.h CACHE INTERNAL "" FORCE)
  set (exceptionhandler_SOURCES ${exceptionhandler_SOURCES} exchndl.cpp  CACHE INTERNAL "" FORCE)
endif ()

#if (WZ_LIBIFY)
#  ${CMAKE_CURRENT_SOURCE_DIR}/add_library(exceptionhandler STATIC ${exceptionhandler_SOURCES})
#  ${CMAKE_CURRENT_SOURCE_DIR}/find_library(EXCEPTIONHANDLER_LIBRARY NAMES exceptionhandler PATHS ${CMAKE_CURRENT_BINARY_DIR} NO_DEFAULT_PATH)
#endif ()
