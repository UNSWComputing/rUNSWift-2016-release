cmake_minimum_required(VERSION 2.8.0 FATAL_ERROR)

set(CTC_DIR $ENV{CTC_DIR})

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 4)

set(CMAKE_C_COMPILER   ${CTC_DIR}/cross/bin/i686-aldebaran-linux-gnu-gcc)

set(CMAKE_FIND_ROOT_PATH
    ${CTC_DIR}/cross
    ${CTC_DIR}
    ${CTC_DIR}/../sysroot_legacy
)
# we may actually want native programs, but i prefer we override that on a FIND_XXX basis
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
