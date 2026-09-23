# Command-line tools. Both carry self-tests that CTest runs when the
# test suites are built.
foreach(tool m3replay m3lockstep)
    add_executable(${tool} ${tool}.c)
    target_link_libraries(${tool} PRIVATE maul3d)
    maul_apply_flags(${tool})
endforeach()
if(MAUL3D_INSTALL)
    install(TARGETS m3replay m3lockstep RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(MAUL3D_BUILD_TESTS)
    add_test(NAME replaytool COMMAND m3replay selftest)
    add_test(NAME replaydiff COMMAND m3replay difftest)
    add_test(NAME lockstep COMMAND m3lockstep)
endif()
