#
# Top-level Makefile — convenience wrapper around CMake.
#
# Real build rules live in src/CMakeLists.txt.
#

BUILD_DIR      ?= build
JOBS           ?= $(shell nproc 2>/dev/null || echo 4)

DB_SRC         := dosbox-staging
DB_BUILD       := $(DB_SRC)/build
PATCHES        := patches/sdlmain-expose-setup.patch \
                  patches/sdlmain-skip-gl-probe-headless.patch \
                  patches/enet-clock-gettime-mingw.patch
PATCH_MARKER   := $(DB_SRC)/.dosiz-patched

# MSYS2 MinGW-w64 quirk: meson's CMake dependency resolver calls
# Python's os.path.expanduser('~'), which on Windows Python consults
# USERPROFILE (not HOME).  MSYS2's profile scripts scrub that var, so
# we re-seed it so zlib-ng (and any other CMake-dep) can resolve.
UNAME_S := $(shell uname -s 2>/dev/null)
ifneq (,$(findstring MINGW,$(UNAME_S))$(findstring MSYS,$(UNAME_S)))
  export USERPROFILE ?= $(shell cygpath -m "$$HOME" 2>/dev/null || echo C:/Users/$(USER))
endif

.PHONY: all build clean distclean configure dosbox patch

all: build

# Apply the patches dosiz maintains on top of dosbox-staging.  Idempotent:
# the marker records that we've applied.  `make distclean` or a manual
# `git checkout .` in the submodule resets it.
patch: $(PATCH_MARKER)

$(PATCH_MARKER): $(PATCHES)
	cd $(DB_SRC) && $(foreach p,$(PATCHES),patch -p1 < ../$(p) &&) true
	touch $@

# Build the dosbox-staging libraries via its own meson build.
dosbox: patch $(DB_BUILD)/libdosbox.a

$(DB_BUILD)/libdosbox.a:
	meson setup $(DB_BUILD) $(DB_SRC) --buildtype=release
	ninja -C $(DB_BUILD)

configure: dosbox
	cmake -S src -B $(BUILD_DIR)

build: configure
	cmake --build $(BUILD_DIR) -j$(JOBS)

clean:
	rm -rf $(BUILD_DIR)

distclean: clean
	cd $(DB_SRC) && git checkout -- .
	rm -f $(PATCH_MARKER)
	rm -rf $(DB_BUILD)

test: build
	cd $(BUILD_DIR) && ctest --output-on-failure
