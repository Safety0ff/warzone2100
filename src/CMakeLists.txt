set (warzone2100_SUBDIRS
  framework
  exceptionhandler
  gamelib
  iniparser
  ivis_opengl
  netplay
  script
  sequence
  sound
  widget
  warzone2100
)

foreach (subdir ${warzone2100_SUBDIRS})
  add_subdirectory (${subdir})
  foreach (source ${${subdir}_SOURCES})
    set (wz_SOURCES ${wz_SOURCES} ${subdir}/${source})
  endforeach ()
  foreach (yacc ${${subdir}_YACC})
    set (wz_YACC ${wz_YACC} ${subdir}/${yacc})
  endforeach ()
  foreach (flex ${${subdir}_FLEX})
    set (wz_FLEX ${wz_FLEX} ${subdir}/${flex})
  endforeach ()
endforeach ()

foreach (y_file ${wz_YACC})
  YACC_TARGET ("${y_file}_parser" ${y_file}_parser.y ${CMAKE_CURRENT_BINARY_DIR}/${y_file}_parser.tab.cpp)
  set (wz_YACC_FLEX_SOURCES ${wz_YACC_FLEX_SOURCES} "${YACC_${y_file}_parser_OUTPUTS}")
  set_source_files_properties ("${YACC_${y_file}_parser_OUTPUTS}" PROPERTIES GENERATED true)
endforeach()

foreach (l_file ${wz_FLEX})
  FLEX_TARGET ("${l_file}_lexer" ${l_file}_lexer.l ${CMAKE_CURRENT_BINARY_DIR}/${l_file}_lexer.lex.cpp)
  set (wz_YACC_FLEX_SOURCES ${wz_YACC_FLEX_SOURCES} "${FLEX_${l_file}_lexer_OUTPUTS}")
  set_source_files_properties ("${FLEX_${l_file}_lexer_OUTPUTS}" PROPERTIES GENERATED true)
  list (FIND warzone2100_YACC l_file index)
  foreach (y_file ${wz_YACC})
    if (y_file STREQUAL l_file)
      set_source_files_properties("${FLEX_${l_file}_lexer_OUTPUTS}"
        PROPERTIES OBJECT_DEPENDS "${YACC_${l_file}_parser_OUTPUT_HEADER}")
      break ()
    endif ()
  endforeach ()
endforeach ()

add_executable (warzone2100 ${wz_SOURCES} ${wz_YACC_FLEX_SOURCES})

add_dependencies (warzone2100 miniupnpc)

include_directories (
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/warzone2100
  ${MINIUPNPC_INCLUDE_DIR}
)

target_link_libraries (warzone2100
  ${ZLIB_LIBRARY}
  ${SDL_LIBRARY}
  ${PNG_LIBRARY}
  ${SDLNET_LIBRARY}
  ${OPENAL_LIBRARY}
  ${PHYSFS_LIBRARY}
  ${GETTEXT_LIBRARIES}
  ${GLC_LIBRARIES}
  ${OPENGL_LIBRARY}
  ${THEORA_LIBRARY}
  ${OGG_LIBRARY}
  ${VORBIS_LIBRARY}
  ${VORBISFILE_LIBRARY}
  ${MINIUPNPC_LIBRARY}
)

if (NEED_DEP_DEPS)
  target_link_libraries (warzone2100
    ${FONTCONFIG_LIBRARIES}
    ${FREETYPE2_LIBRARIES}
    ${ICONV_LIBRARIES}
    ${EXPAT_LIBRARY}
  )
endif (NEED_DEP_DEPS)

if (WIN32)
  target_link_libraries(warzone2100
    ${WINMM_LIBRARY}
    ${WINSOCK2_LIBRARY}
    ${BFD_LIBRARY}
    ${DBGHELP_LIBRARY}
    ${IBERTY_LIBRARY}
    ${SHFOLDER_LIBRARY}
    ${SHLWAPI_LIBRARY}
    ${PSAPI_LIBRARY}
)
else ()
  target_link_libraries(warzone2100
    ${JPEG_LIBRARY}
)
endif ()
