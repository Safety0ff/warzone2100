set (ivis_comon_HEADERS
  bitimage.h
  imd.h
  ivi.h
  ivisdef.h
  jpeg_encoder.h
  pieblitfunc.h
  pieclip.h
  piedef.h
  piefunc.h
  piemode.h
  piepalette.h
  piestate.h
  pietypes.h
  png_util.h
  rendmode.h
  tex.h
  textdraw.h
  CACHE INTERNAL ""
)

set (ivis_common_SOURCES
  bitimage.cpp
  imd.cpp
  imdload.cpp
  jpeg_encoder.cpp
  pieclip.cpp
  piestate.cpp
  png_util.cpp
  CACHE INTERNAL ""
)

#libivis_common_a_LIBADD = $(top_builddir)/lib/framework/libframework.a
