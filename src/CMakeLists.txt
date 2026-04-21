#
# dosemu/src/CMakeLists.txt
#
# Builds the dosemu CLI binary. It links against dosbox-staging (built as a
# static library in the parent tree, ../dosbox-staging) plus the thin bridge
# and translation layers in this directory.
#
# This file is a skeleton — sources are added as Phase 1+ work lands. For now
# it just defines the target and project-level options so `cmake -S src -B build`
# does not fail on a fresh clone.
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
# dosbox-staging (git submodule at ../dosbox-staging, fetched during Phase 0)
# ---------------------------------------------------------------------------
set(DOSBOX_DIR "${CMAKE_SOURCE_DIR}/../dosbox-staging" CACHE PATH
    "Path to dosbox-staging checkout")

if(EXISTS "${DOSBOX_DIR}/CMakeLists.txt")
  # Build dosbox as static lib.  The exact target name will be set once
  # the submodule is wired in (Phase 1).
  # add_subdirectory(${DOSBOX_DIR} dosbox-build EXCLUDE_FROM_ALL)
  message(STATUS "dosbox-staging found at ${DOSBOX_DIR}")
else()
  message(WARNING
    "dosbox-staging not found at ${DOSBOX_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

# ---------------------------------------------------------------------------
# dosemu CLI (placeholder — real sources land in Phase 1)
# ---------------------------------------------------------------------------
add_executable(dosemu dosemu.cc)
# target_link_libraries(dosemu PRIVATE dosbox)
