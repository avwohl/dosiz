#
# dosiz/src/CMakeLists.txt
#
cmake_minimum_required(VERSION 3.25)
project(dosiz CXX C)

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
# On Mac (Homebrew) glib lives in /opt/homebrew; pkg_check_modules's plain
# ${GLIB_LIBRARIES} gives bare library names — we need ${GLIB_LDFLAGS} (which
# includes -L paths) and the cflags for headers.
link_directories(${GLIB_LIBRARY_DIRS})
include_directories(${GLIB_INCLUDE_DIRS})

add_executable(dosiz
  dosiz.cc
  config.cc
  bridge.cc
  debug_settings.cpp
)

# dosbox-staging searches $exe/../Resources (macOS) or $exe/resources
# (other) for shaders, fonts, translation files, and Unicode
# codepage-mapping tables.  Symlink the committed
# dosbox-staging/Resources/ into BOTH spots so the lookup succeeds on
# every platform without a full install.
add_custom_command(TARGET dosiz POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          ${DB}/Resources
          $<TARGET_FILE_DIR:dosiz>/resources
  COMMAND ${CMAKE_COMMAND} -E create_symlink
          ${DB}/Resources
          $<TARGET_FILE_DIR:dosiz>/../Resources
  VERBATIM)

target_include_directories(dosiz PRIVATE
  ${DB}/include
  ${DB_BUILD}                    # generated config.h
  ${DB}/src/libs/loguru          # loguru.hpp for verbosity control
  ${SDL2_INCLUDE_DIRS}
)

set(DB_ARCHIVES
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
)

if(APPLE)
  # Mac: ld64 resolves circular deps via multi-pass, no --start-group needed.
  # Subprojects are system libs on Mac (except mt32emu, which meson built).
  target_link_libraries(dosiz PRIVATE
    ${DB_ARCHIVES}
    ${DB_BUILD}/subprojects/munt-libmt32emu_2_7_1/libmt32emu.dylib
    SDL2::SDL2
    ${GLIB_LIBRARIES}
    dl m
  )

  find_library(SPEEX speexdsp REQUIRED)
  find_library(OPUSFILE opusfile REQUIRED)
  find_library(IIR iir REQUIRED)
  find_library(PNG png REQUIRED)
  target_link_libraries(dosiz PRIVATE
    ${SPEEX} ${OPUSFILE} ${IIR} ${PNG} z
    "-framework CoreFoundation"
    "-framework CoreAudio"
    "-framework AudioUnit"
    "-framework AudioToolbox"
    "-framework CoreMIDI"
    "-framework IOKit"
    "-framework OpenGL"
  )
  # rpath so the mt32emu dylib is found at runtime next to the executable.
  set_target_properties(dosiz PROPERTIES
    BUILD_RPATH "${DB_BUILD}/subprojects/munt-libmt32emu_2_7_1"
    INSTALL_RPATH "@loader_path/../dosbox-staging/build/subprojects/munt-libmt32emu_2_7_1"
  )
else()
  # Linux: GNU ld + --start-group for circular archive refs.
  target_link_libraries(dosiz PRIVATE
    -Wl,--start-group
    ${DB_ARCHIVES}
    ${DB_BUILD}/subprojects/libpng-1.6.40/libpng16.a
    ${DB_BUILD}/subprojects/zlib-ng/libzlib.a
    ${DB_BUILD}/subprojects/munt-libmt32emu_2_7_1/libmt32emu.so
    ${DB_BUILD}/subprojects/iir1-1.9.3/iir/libiir.so
    -Wl,--end-group
    SDL2::SDL2
    ${GLIB_LIBRARIES}
    atomic stdc++fs dl m
  )

  find_library(SDL2_NET SDL2_net REQUIRED)
  find_library(SLIRP slirp REQUIRED)
  find_library(SPEEX speexdsp REQUIRED)
  find_library(OPUSFILE opusfile REQUIRED)
  find_library(FLUIDSYNTH fluidsynth REQUIRED)
  find_library(ASOUND asound REQUIRED)
  find_library(XI Xi REQUIRED)
  find_library(GL GL REQUIRED)
  target_link_libraries(dosiz PRIVATE
    ${SDL2_NET} ${SLIRP} ${SPEEX} ${OPUSFILE}
    ${FLUIDSYNTH} ${ASOUND} ${XI} ${GL}
  )
endif()
