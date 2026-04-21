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
# dosbox-staging
#
# We link against the static libraries produced by dosbox-staging's upstream
# meson build, which lives at ../dosbox-staging/build.  Meson is expected to
# have been run already (see README for the one-time setup step).
#
# Strict cpmemu-style DOS replacement is the end goal: eventually we will
# stop linking libdos.a and libshell.a.  For now every lib gets pulled in so
# we can confirm the mechanical linking works.
# ---------------------------------------------------------------------------
set(DB ${CMAKE_SOURCE_DIR}/../dosbox-staging)
set(DB_BUILD ${DB}/build)

if(NOT EXISTS ${DB_BUILD}/libdosbox.a)
  message(FATAL_ERROR
    "dosbox-staging has not been built.  Run:\n"
    "    meson setup ${DB}/build --buildtype=release ${DB}\n"
    "    ninja -C ${DB}/build")
endif()

find_package(SDL2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)

add_executable(dosemu
  dosemu.cc
  config.cc
  bridge.cc
)

target_include_directories(dosemu PRIVATE
  ${DB}/include
  ${DB_BUILD}                    # generated config.h
  ${DB}/src/libs/loguru          # loguru.hpp for verbosity control
  ${SDL2_INCLUDE_DIRS}
)

# Link order mirrors what meson uses for the dosbox binary.  --start-group /
# --end-group handles circular references between the per-subdir archives
# (e.g. libmidi -> libhardware -> libmidi).
target_link_libraries(dosemu PRIVATE
  -Wl,--start-group
  ${DB_BUILD}/libdosbox.a
  ${DB_BUILD}/src/capture/libcapture.a
  ${DB_BUILD}/src/cpu/libcpu.a
  ${DB_BUILD}/src/dos/libdos.a
  ${DB_BUILD}/src/fpu/libfpu.a
  ${DB_BUILD}/src/gui/libgui.a
  ${DB_BUILD}/src/hardware/libhardware.a
  ${DB_BUILD}/src/ints/libints.a
  ${DB_BUILD}/src/midi/libmidi.a
  ${DB_BUILD}/src/misc/libmisc.a
  ${DB_BUILD}/src/shell/libshell.a
  ${DB_BUILD}/src/libs/ESFMu/libesfmu.a
  ${DB_BUILD}/src/libs/YM7128B_emu/libym7128bemu.a
  ${DB_BUILD}/src/libs/decoders/libdecoders.a
  ${DB_BUILD}/src/libs/loguru/libloguru.a
  ${DB_BUILD}/src/libs/manymouse/libmanymouse.a
  ${DB_BUILD}/src/libs/nuked/libnuked.a
  ${DB_BUILD}/src/libs/residfp/libresidfp.a
  ${DB_BUILD}/src/libs/tal-chorus/libtalchorus.a
  ${DB_BUILD}/src/libs/whereami/libwhereami.a
  ${DB_BUILD}/src/libs/zmbv/libzmbv.a
  ${DB_BUILD}/subprojects/libpng-1.6.40/libpng16.a
  ${DB_BUILD}/subprojects/zlib-ng/libzlib.a
  ${DB_BUILD}/subprojects/munt-libmt32emu_2_7_1/libmt32emu.so
  ${DB_BUILD}/subprojects/iir1-1.9.3/iir/libiir.so
  -Wl,--end-group
  SDL2::SDL2
  ${GLIB_LIBRARIES}
  atomic stdc++fs dl m
)

# Host libs that meson's dosbox also links against.
find_library(SDL2_NET SDL2_net REQUIRED)
find_library(SLIRP slirp REQUIRED)
find_library(SPEEX speexdsp REQUIRED)
find_library(OPUSFILE opusfile REQUIRED)
find_library(FLUIDSYNTH fluidsynth REQUIRED)
find_library(ASOUND asound REQUIRED)
find_library(XI Xi REQUIRED)
find_library(GL GL REQUIRED)
target_link_libraries(dosemu PRIVATE
  ${SDL2_NET} ${SLIRP} ${SPEEX} ${OPUSFILE}
  ${FLUIDSYNTH} ${ASOUND} ${XI} ${GL}
)
