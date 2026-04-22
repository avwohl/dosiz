#
# Top-level Makefile — convenience wrapper around CMake.
#
# Real build rules live in src/CMakeLists.txt.
#

BUILD_DIR      ?= build
JOBS           ?= $(shell nproc 2>/dev/null || echo 4)

DB_SRC         := dosbox-staging
DB_BUILD       := $(DB_SRC)/build
PATCH          := patches/sdlmain-expose-setup.patch
PATCH_MARKER   := $(DB_SRC)/.dosemu-patched

.PHONY: all build clean distclean configure dosbox patch

all: build

# Apply the sdlmain un-static patch into the submodule.  Idempotent: the
# marker file records that we've applied; `make distclean` or a manual
# `git checkout .` in the submodule resets it.
patch: $(PATCH_MARKER)

$(PATCH_MARKER): $(PATCH)
	cd $(DB_SRC) && patch -p1 < ../$(PATCH)
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
