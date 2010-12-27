set (script_HEADERS
  chat_processing.h
  codeprint.h
  event.h
  eventsave.h
  interpreter.h
  parse.h
  script.h
  stack.h
  CACHE INTERNAL ""
)

set (script_SOURCES
  codeprint.cpp
  event.cpp
  eventsave.cpp
  interpreter.cpp
  script.cpp
  stack.cpp
  CACHE INTERNAL ""
)

set (script_YACC
  chat
  script
  CACHE INTERNAL ""
)

set (script_FLEX
  chat
  script
  CACHE INTERNAL ""
)

#libscript_a_LIBADD = $(top_builddir)/lib/framework/libframework.a
