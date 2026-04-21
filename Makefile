#
# Top-level Makefile — convenience wrapper around CMake.
#
# Real build rules live in src/CMakeLists.txt. This file just provides the
# common make targets (build, clean, test) for people used to cpmemu's
# src/makefile workflow.
#

BUILD_DIR ?= build
JOBS      ?= $(shell nproc 2>/dev/null || echo 4)

.PHONY: all build clean test configure

all: build

configure:
	cmake -S src -B $(BUILD_DIR)

build: configure
	cmake --build $(BUILD_DIR) -j$(JOBS)

clean:
	rm -rf $(BUILD_DIR)

test: build
	cd $(BUILD_DIR) && ctest --output-on-failure
