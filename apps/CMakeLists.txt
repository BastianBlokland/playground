# --------------------------------------------------------------------------------------------------
# Applications cmake file.
# --------------------------------------------------------------------------------------------------

add_executable(smoke smoke.c)
target_link_libraries(smoke PRIVATE app_ecs asset gap rend ui log)
