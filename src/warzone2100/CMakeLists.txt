set (warzone2100_HEADERS
  action.h
  actiondef.h
  advvis.h
  ai.h
  anim_id.h
  astar.h
  atmos.h
  basedef.h
  baseobject.h
  bridge.h
  bucket3d.h
  cheat.h
  challenge.h
  clparse.h
  cluster.h
  cmddroiddef.h
  cmddroid.h
  combat.h
  component.h
  configuration.h
  console.h
  data.h
  design.h
  difficulty.h
  display3ddef.h
  display3d.h
  displaydef.h
  display.h
  drive.h
  droiddef.h
  droid.h
  e3demo.h
  edit3d.h
  effects.h
  featuredef.h
  feature.h
  fpath.h
  frend.h
  frontend.h
  functiondef.h
  function.h
  game.h
  gateway.h
  geometry.h
  group.h
  hci.h
  ingameop.h
  init.h
  intdisplay.h
  intelmap.h
  intfac.h
  intimage.h
  intorder.h
  keybind.h
  keyedit.h
  keymap.h
  levelint.h
  levels.h
  lighting.h
  loadsave.h
  loop.h
  main.h
  mapdisplay.h
  mapgrid.h
  map.h
  mechanics.h
  messagedef.h
  message.h
  messagely.h
  miscimd.h
  missiondef.h
  mission.h
  modding.h
  movedef.h
  move.h
  multigifts.h
  multiint.h
  multijoin.h
  multilimit.h
  multimenu.h
  multiplay.h
  multirecv.h
  multistat.h
  objectdef.h
  objects.h
  objmem.h
  oprint.h
  orderdef.h
  order.h
  parsetest.h
  pointtree.h
  positiondef.h
  power.h
  projectiledef.h
  projectile.h
  radar.h
  random.h
  raycast.h
  researchdef.h
  research.h
  scores.h
  scriptai.h
  scriptcb.h
  scriptextern.h
  scriptfuncs.h
  scriptobj.h
  scripttabs.h
  scriptvals.h
  selection.h
  seqdisp.h
  statsdef.h
  stats.h
  stringdef.h
  structuredef.h
  structure.h
  terrain.h
  text.h
  texture.h
  transporter.h
  visibility.h
  version.h
  warcam.h
  warzoneconfig.h
  wavecast.h
  weapondef.h
  wrappers.h
  CACHE INTERNAL ""
)

set (warzone2100_SOURCES
  action.cpp
  advvis.cpp
  ai.cpp
  astar.cpp
  atmos.cpp
  aud.cpp
  baseobject.cpp
  bridge.cpp
  bucket3d.cpp
  challenge.cpp
  cheat.cpp
  clparse.cpp
  cluster.cpp
  cmddroid.cpp
  combat.cpp
  component.cpp
  configuration.cpp
  console.cpp
  data.cpp
  design.cpp
  difficulty.cpp
  display3d.cpp
  display.cpp
  drive.cpp
  droid.cpp
  e3demo.cpp
  edit3d.cpp
  effects.cpp
  feature.cpp
  fpath.cpp
  frontend.cpp
  function.cpp
  game.cpp
  gateway.cpp
  geometry.cpp
  group.cpp
  hci.cpp
  ingameop.cpp
  init.cpp
  intdisplay.cpp
  intelmap.cpp
  intimage.cpp
  intorder.cpp
  keybind.cpp
  keyedit.cpp
  keymap.cpp
  levels.cpp
  lighting.cpp
  loadsave.cpp
  loop.cpp
  main.cpp
  map.cpp
  mapdisplay.cpp
  mapgrid.cpp
  mechanics.cpp
  message.cpp
  miscimd.cpp
  mission.cpp
  move.cpp
  multibot.cpp
  multigifts.cpp
  multiint.cpp
  multijoin.cpp
  multilimit.cpp
  multimenu.cpp
  multiopt.cpp
  multiplay.cpp
  multistat.cpp
  multistruct.cpp
  multisync.cpp
  objects.cpp
  objmem.cpp
  oprint.cpp
  order.cpp
  parsetest.cpp
  pointtree.cpp
  power.cpp
  projectile.cpp
  radar.cpp
  random.cpp
  raycast.cpp
  research.cpp
  scores.cpp
  scriptai.cpp
  scriptcb.cpp
  scriptextern.cpp
  scriptfuncs.cpp
  scriptobj.cpp
  scripttabs.cpp
  scriptvals.cpp
  selection.cpp
  seqdisp.cpp
  stats.cpp
  structure.cpp
  terrain.cpp
  text.cpp
  texture.cpp
  transporter.cpp
  version.cpp
  visibility.cpp
  warcam.cpp
  warzoneconfig.cpp
  wavecast.cpp
  wrappers.cpp
  CACHE INTERNAL ""
)

set (warzone2100_YACC
  message
  scriptvals
  CACHE INTERNAL ""
)

set (warzone2100_FLEX
  message
  scriptvals
  level
  CACHE INTERNAL ""
)
#  autorevision.h ?

#warzone2100_LDADD =
#  $(top_builddir)/lib/widget/libwidget.a
#  $(top_builddir)/lib/sequence/libsequence.a
#  $(top_builddir)/lib/sound/libsound.a
#  $(top_builddir)/lib/script/libscript.a
#  $(top_builddir)/lib/netplay/libnetplay.a
#  $(top_builddir)/lib/netplay/miniupnpc/libminiupnpc.a
#  $(top_builddir)/lib/ivis_opengl/libivis_opengl.a
#  $(top_builddir)/lib/ivis_common/libivis_common.a
#  $(top_builddir)/lib/gamelib/libgamelib.a
#  $(top_builddir)/lib/framework/libframework.a
#  $(top_builddir)/lib/iniparser/libiniparser.a
#  $(top_builddir)/lib/exceptionhandler/libexceptionhandler.a
