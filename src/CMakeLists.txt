#
# dosemu/src/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.25)
project(dosemu CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# ---------------------------------------------------------------------------
# dosemu CLI — subprocess-style wrapper around dosbox-staging.
#
# The dosbox-staging submodule lives at ${CMAKE_SOURCE_DIR}/../dosbox-staging
# and is built separately via its own meson build system. At runtime dosemu
# locates the dosbox binary via:
#   1. --dosbox=PATH on the CLI
#   2. DOSEMU_DOSBOX env var
#   3. ../dosbox-staging/build/dosbox relative to the dosemu binary
#   4. dosbox / dosbox-staging on PATH
# ---------------------------------------------------------------------------
add_executable(dosemu
  dosemu.cc
  config.cc
  dosbox_conf.cc
)

# Handy make target: build the bundled dosbox-staging if it hasn't been.
add_custom_target(dosbox
  COMMAND ${CMAKE_COMMAND} -E echo "Building dosbox-staging via meson..."
  COMMAND meson setup --buildtype=release ${CMAKE_SOURCE_DIR}/../dosbox-staging/build ${CMAKE_SOURCE_DIR}/../dosbox-staging || true
  COMMAND ninja -C ${CMAKE_SOURCE_DIR}/../dosbox-staging/build
  VERBATIM
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()
add_test(NAME smoke
  COMMAND bash ${CMAKE_SOURCE_DIR}/../tests/run_tests.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..)
set_tests_properties(smoke PROPERTIES
  ENVIRONMENT "DOSEMU=$<TARGET_FILE:dosemu>")
