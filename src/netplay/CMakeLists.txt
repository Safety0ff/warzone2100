set (netplay_HEADERS
  netlog.h
  netplay.h
  netqueue.h
  netsocket.h
  nettypes.h
  CACHE INTERNAL ""
)

set (netplay_SOURCES
  netjoin_stub.cpp
  netlog.cpp
  netplay.cpp
  netqueue.cpp
  netsocket.cpp
  nettypes.cpp
  CACHE INTERNAL ""
)

#libnetplay_a_LIBADD = $(top_builddir)/lib/framework/libframework.a $(top_builddir)/lib/netplay/miniupnpc/libminiupnpc.a
