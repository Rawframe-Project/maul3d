# Command-line tools. Both carry self-tests that CTest runs when the
# test suites are built.
foreach(tool m3replay m3lockstep)
    add_executable(${tool} ${tool}.c)
    target_link_libraries(${tool} PRIVATE maul3d)
    maul_apply_flags(${tool})
    # The tools read and write files with standard fopen, which the Windows
    # C runtime reports as deprecated in favor of its own fopen_s.
    if(WIN32)
        target_compile_definitions(${tool} PRIVATE _CRT_SECURE_NO_WARNINGS)
    endif()
endforeach()
if(MAUL3D_INSTALL)
    install(TARGETS m3replay m3lockstep RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(MAUL3D_BUILD_TESTS)
    add_test(NAME replaytool COMMAND m3replay selftest)
    add_test(NAME replaydiff COMMAND m3replay difftest)
    add_test(NAME replaysize COMMAND m3replay sizetest)
    add_test(NAME lockstep COMMAND m3lockstep)
    # Each labeled by its own name, as maul_add_test labels a suite.
    foreach(name replaytool replaydiff replaysize lockstep)
        set_tests_properties(${name} PROPERTIES LABELS ${name})
    endforeach()
endif()
