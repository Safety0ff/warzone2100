set (sound_HEADERS
  aud.h
  audio.h
  audio_id.h
  cdaudio.h
  mixer.h
  playlist.h
  oggvorbis.h
  openal_error.h
  track.h
  tracklib.h
  CACHE INTERNAL ""
)

set (sound_SOURCES
  audio.cpp
  audio_id.cpp
  cdaudio.cpp
  oggvorbis.cpp
  openal_error.cpp
  openal_track.cpp
  playlist.cpp
  track.cpp
  CACHE INTERNAL ""
)

#libsound_a_LIBADD = $(top_builddir)/lib/gamelib/libgamelib.a
#  $(top_builddir)/lib/framework/libframework.a
