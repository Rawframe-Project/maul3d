# In-tree samples: small programs built with the library. The minimal/
# subdirectory is a separate project that consumes the installed
# package instead; CI builds it against a fresh install.

foreach(sample stack rollback car character)
    add_executable(sample_${sample} ${sample}.c)
    target_link_libraries(sample_${sample} PRIVATE maul3d)
    maul_apply_flags(sample_${sample})
endforeach()
