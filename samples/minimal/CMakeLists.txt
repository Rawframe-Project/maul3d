# A separate project that knows nothing about the maul3d source tree
# and consumes only the installed package. CI installs the library and
# builds this; if it builds, links and runs, the package works.
cmake_minimum_required(VERSION 3.22)
project(maul3d_consumer LANGUAGES C)

find_package(maul3d 0.0 REQUIRED CONFIG)

add_executable(consumer consumer.c)
target_link_libraries(consumer PRIVATE maul3d::maul3d)
