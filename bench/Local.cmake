# The benchmark scenes, shared with test/test_bench.c.
target_sources(${PROJECT_NAME}_bench PRIVATE scenes.c)
# The scenes build through the suites' helpers (test/helpers.h).
target_include_directories(${PROJECT_NAME}_bench PRIVATE ${PROJECT_SOURCE_DIR}/test)
