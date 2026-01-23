# --------------------------------------------------------------------------------------------------
# Development library cmake file.
# --------------------------------------------------------------------------------------------------

add_library(dev STATIC
  src/asset.c
  src/ecs.c
  src/finder.c
  src/input.c
  src/interface.c
  src/log_viewer.c
  src/menu.c
  src/panel.c
  src/register.c
  src/rend.c
  src/shape.c
  src/sound.c
  src/stats.c
  src/text.c
  src/trace.c
  src/widget.c
  )
target_include_directories(dev PUBLIC include)
target_link_libraries(dev PUBLIC core ecs scene log)
target_link_libraries(dev PRIVATE gap rend input ui snd trace)
