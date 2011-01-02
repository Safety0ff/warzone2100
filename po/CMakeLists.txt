# Available languages.
#first read in the space separated list
file (STRINGS LINGUAS LANGUAGES)
#convert all newlines into ; to have a nice "set"
separate_arguments (LANGUAGES)

############ Validation. ###########
# TODO Put some of this stuff in the FindGettext.cmake module
execute_process (
  COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} --version
  OUTPUT_VARIABLE GETTEXT_MSGFMT_VERSION
)
string (REGEX MATCH GNU GETTEXT_MSGFMT_GNU ${GETTEXT_MSGFMT_VERSION})
if (GETTEXT_MSGFMT_GNU)
  set (MSGFMT_EXTENSION gmo)
  set (MSGFMT_OPTIONS ${MSGFMT_OPTIONS} --statistics)
else ()
  set (MSGFMT_EXTENSION mo)
endif ()

set (MSGMERGE_OPTIONS --no-wrap --width=1 --quiet -s)

find_program (GETTEXT_MSGINIT_EXECUTABLE msginit)
if (NOT GETTEXT_MSGINIT_EXECUTABLE)
  message (FATAL_ERROR "msginit not found")
endif (NOT GETTEXT_MSGINIT_EXECUTABLE)

find_program (GETTEXT_XGETTEXT_EXECUTABLE xgettext)
if (NOT GETTEXT_XGETTEXT_EXECUTABLE )
  message (FATAL_ERROR "xgettext not found")
endif (NOT GETTEXT_XGETTEXT_EXECUTABLE )

find_program (GETTEXT_MSGCAT_EXECUTABLE msgcat)
if (NOT GETTEXT_MSGCAT_EXECUTABLE )
  message (FATAL_ERROR "msgcat not found")
endif (NOT GETTEXT_MSGCAT_EXECUTABLE )

find_program (GETTEXT_MSGATTRIB_EXECUTABLE msgattrib)
if (NOT GETTEXT_MSGATTRIB_EXECUTABLE)
      message (FATAL_ERROR "msgattrib not found")
endif (NOT GETTEXT_MSGATTRIB_EXECUTABLE)

############ POTFILES ###########

add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/POTFILES
  COMMAND po/update-po.sh ${CMAKE_CURRENT_BINARY_DIR}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
set_source_files_properties (${CMAKE_CURRENT_BINARY_DIR}/POTFILES PROPERTIES GENERATED TRUE)

############ warzone2100.pot ###########

add_custom_command (
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot
  COMMAND ${GETTEXT_XGETTEXT_EXECUTABLE}
    --default-domain=warzone2100
    --add-comments=TRANSLATORS:
    --from-code=UTF-8
    --no-wrap
    --width=1
    --keyword=_
    --keyword=N_
    --keyword=P_:1c,2
    --keyword=NP_:1c,2
    --files-from=${CMAKE_CURRENT_BINARY_DIR}/POTFILES
    --copyright-holder="Warzone Resurrection Project"
    --package-name=warzone2100
    --msgid-bugs-address=warzone-dev@gna.org
    --output=${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot
  COMMAND sed -i
    -f ${CMAKE_CURRENT_SOURCE_DIR}/remove-potcdate.sin
    ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/POTFILES ${CMAKE_CURRENT_SOURCE_DIR}/remove-potcdate.sin
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  VERBATIM
)
set_source_files_properties (${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot PROPERTIES GENERATED TRUE)

############ *.po *.gmo ###########

foreach (_lang ${LANGUAGES})
  if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po)
    message (WARNING "File ${_lang}.po does not exist. If you are a translator, you can create it through 'msginit'.")
  else ()
    if (${CMAKE_CURRENT_BINARY_DIR} EQUAL ${CMAKE_CURRENT_SOURCE_DIR} OR PO_UPDATE)
      set (_msgmerge_output ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po)
      add_custom_command (
        OUTPUT ${_msgmerge_output}
        COMMAND ${GETTEXT_MSGMERGE_EXECUTABLE} --update ${MSGMERGE_OPTIONS} --backup=none ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po
        COMMENT "Updating ${_lang}.po"
      )
    else ()
      set (_msgmerge_output ${CMAKE_CURRENT_BINARY_DIR}/${_lang}.po)
      add_custom_command (
        OUTPUT ${_msgmerge_output}
        COMMAND ${GETTEXT_MSGMERGE_EXECUTABLE} ${MSGMERGE_OPTIONS} -o ${_msgmerge_output} ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/warzone2100.pot ${CMAKE_CURRENT_SOURCE_DIR}/${_lang}.po
        COMMENT "Generating updated ${_lang}.po"
      )
      set_source_files_properties (${_msgmerge_output} PROPERTIES GENERATED TRUE)
    endif ()
  endif ()

  add_custom_command (
     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_lang}.${MSGFMT_EXTENSION}
     COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} -o ${CMAKE_CURRENT_BINARY_DIR}/${_lang}.${MSGFMT_EXTENSION} ${_msgmerge_output}
     DEPENDS ${_msgmerge_output}
  )

  list (APPEND mo_FILES ${CMAKE_CURRENT_BINARY_DIR}/${_lang}.${MSGFMT_EXTENSION})
  list (APPEND _msgmerge_outputs ${_msgmerge_output})

endforeach (_lang)

if (${CMAKE_CURRENT_BINARY_DIR} EQUAL ${CMAKE_CURRENT_SOURCE_DIR} OR PO_UPDATE)
  # Make sure we don't remove the original po files with make clean.
  # This breaks make clean, so we manually add all the files to be cleaned.
  set_directory_properties (PROPERTY CLEAN_NO_CUSTOM TRUE)
  set_property (DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${mo_FILES} warzone2100.pot POTFILES)
endif ()

############ Build targets ###########

# I don't think there's a point to disable this target, just add this target to all on ENABLE_NLS
if (ENABLE_NLS)
  set (_all ALL)
endif ()
add_custom_target (translations ${_all} DEPENDS ${mo_FILES})

if (PO_UPDATE)
  # This target doesn't make sense without PO_UPDATE
  add_custom_target (po-update DEPENDS ${_msgmerge_outputs})
endif ()
