set (gamelib_HEADERS
  anim.h
  animobj.h
  gtime.h
  hashtable.h
  parser.h
  CACHE INTERNAL ""
)

set (gamelib_SOURCES
  anim.cpp
  animobj.cpp
  gtime.cpp
  hashtable.cpp
  CACHE INTERNAL ""
)

set (gamelib_FLEX
  audp
  CACHE INTERNAL ""
)

set (gamelib_YACC
  audp
  CACHE INTERNAL ""
)

#WZ_YACC_FLEX ("${gamelib_YACC}" "${gamelib_FLEX}" gamelib_SOURCES)

#libgamelib_a_LIBADD = $(top_builddir)/lib/ivis_opengl/libivis_opengl.a
#  $(top_builddir)/lib/ivis_common/libivis_common.a $(top_builddir)/lib/framework/libframework.a

