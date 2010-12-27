#AM_CFLAGS = $(WZ_CFLAGS) -Wno-missing-declarations

set (ivis_opengl_HEADERS
  piematrix.h
  screen.h
  CACHE INTERNAL ""
)

set (ivis_opengl_SOURCES
  ivi.cpp
  pieblitfunc.cpp
  piedraw.cpp
  piefunc.cpp
  piematrix.cpp
  piemode.cpp
  piepalette.cpp
  piestate.cpp
  rendmode.cpp
  screen.cpp
  tex.cpp
  textdraw.cpp
  CACHE INTERNAL ""
)

if (NOT SYSTEM_GLEE)
  set (ivis_opengl_HEADERS GLee.h ${ivis_opengl_HEADERS} CACHE INTERNAL "" FORCE)
  set (ivis_opengl_SOURCES GLee.c ${ivis_opengl_SOURCES} CACHE INTERNAL "" FORCE)
endif (NOT SYSTEM_GLEE)

#libivis_opengl_a_LIBADD = $(top_builddir)/lib/ivis_common/libivis_common.a
#  $(top_builddir)/lib/framework/libframework.a
