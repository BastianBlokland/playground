# --------------------------------------------------------------------------------------------------
# Applications cmake file.
# --------------------------------------------------------------------------------------------------

add_executable(smoke smoke.c)
target_link_libraries(smoke PRIVATE app_ecs asset gap rend ui log)

add_executable(pack pack.c)
target_link_libraries(pack PRIVATE app_ecs log asset)
